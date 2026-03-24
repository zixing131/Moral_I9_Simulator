#include "cbfs_host.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#ifdef _WIN32
#include <stdlib.h> /* _fullpath */
#endif

#ifndef MORAL_CBFS_HOST_LOG_MAX
#define MORAL_CBFS_HOST_LOG_MAX 32u
#endif

#define CBFS_HOST_SLOTS 10
#define CBFS_PATH_UWCHAR_MAX 512
#define CBFS_NAND_NEEDLE_CAP 16

typedef struct
{
    FILE *fp;
    u8 in_use;
} cbfs_host_slot_t;

static cbfs_host_slot_t g_cbfs_host_slots[CBFS_HOST_SLOTS];

static u32 g_cbfs_host_log_count;

static int moral_cbfs_read_guest_ucs2_z(u32 gva, u16 *wbuf, int max_wchars)
{
    int i;

    if (wbuf == NULL || max_wchars < 2)
        return -1;
    for (i = 0; i < max_wchars - 1; i++)
    {
        u16 ch;
        if (uc_mem_read(MTK, gva + (u32)i * 2u, &ch, 2) != UC_ERR_OK)
            return -1;
        wbuf[i] = ch;
        if (ch == 0u)
            return i;
    }
    wbuf[max_wchars - 1] = 0u;
    return max_wchars - 1;
}

/* BMP UCS-2LE → UTF-8，仅用于路径 */
static int moral_ucs2_to_utf8_path(const u16 *w, char *out, size_t cap)
{
    size_t o = 0u;

    if (out == NULL || cap < 4u)
        return -1;
    while (*w != 0u && o + 4u < cap)
    {
        u32 c = (u32)*w++;

        if (c < 0x80u)
            out[o++] = (char)c;
        else if (c < 0x800u)
        {
            out[o++] = (char)(0xC0u | (c >> 6));
            out[o++] = (char)(0x80u | (c & 0x3Fu));
        }
        else
        {
            out[o++] = (char)(0xE0u | (c >> 12));
            out[o++] = (char)(0x80u | ((c >> 6) & 0x3Fu));
            out[o++] = (char)(0x80u | (c & 0x3Fu));
        }
    }
    out[o] = '\0';
    return (int)o;
}

static int moral_path_has_dotdot(const char *p)
{
    const char *s;

    if (p == NULL)
        return 1;
    for (s = p; *s != '\0'; s++)
    {
        if (s[0] == '.' && s[1] == '.' && (s[2] == '/' || s[2] == '\\' || s[2] == '\0'))
            return 1;
    }
    return 0;
}

static const u8 *moral_memmem(const u8 *hay, size_t hay_len, const u8 *ndl, size_t ndl_len)
{
    size_t i, j;

    if (ndl_len == 0u || hay_len < ndl_len)
        return NULL;
    for (i = 0; i + ndl_len <= hay_len; i++)
    {
        for (j = 0; j < ndl_len; j++)
        {
            if (hay[i + j] != ndl[j])
                break;
        }
        if (j == ndl_len)
            return hay + i;
    }
    return NULL;
}

static const char *moral_cbfs_basename_utf8(const char *p)
{
    const char *a;
    const char *b;
    const char *m;

    if (p == NULL)
        return "";
    a = strrchr(p, '/');
    b = strrchr(p, '\\');
    m = a;
    if (b != NULL && (m == NULL || b > m))
        m = b;
    return m != NULL ? (m + 1) : p;
}

static u32 moral_bmp_file_size_le(const u8 *bmp, size_t rem)
{
    u32 fsz;

    if (rem < 6u)
        return 0u;
    if (bmp[0] != 0x42u || bmp[1] != 0x4Du)
        return 0u;
    fsz = (u32)bmp[2] | ((u32)bmp[3] << 8) | ((u32)bmp[4] << 16) | ((u32)bmp[5] << 24);
    if (fsz < 54u || fsz > (16u * 1024u * 1024u))
        return 0u;
    return fsz;
}

/* 在路径名命中点附近找 BMP 头（资源可能在路径后较远位置） */
static int moral_cbfs_find_bmp_near_path(const u8 *hay, size_t hay_len, size_t path_off, u32 *out_off, u32 *out_sz)
{
    size_t lo;
    size_t hi;
    size_t z;

    if (path_off > hay_len)
        return 0;
    lo = path_off > 8192u ? path_off - 8192u : 0u;
    hi = path_off + (256u * 1024u);
    if (hi > hay_len)
        hi = hay_len;
    for (z = lo; z + 54u <= hi; z++)
    {
        if (hay[z] != 0x42u || hay[z + 1u] != 0x4Du)
            continue;
        u32 fs = moral_bmp_file_size_le(hay + z, hay_len - z);
        if (fs != 0u && z + (size_t)fs <= hay_len)
        {
            *out_off = (u32)z;
            *out_sz = fs;
            return 1;
        }
    }
    return 0;
}

/* rel 是否已带 CARD/、NAND/（大小写不敏感） */
static int moral_cbfs_rel_has_vol_prefix(const char *rel)
{
    char u4[5];
    size_t i;

    if (rel == NULL)
        return 0;
    for (i = 0u; i < 4u && rel[i] != '\0'; i++)
    {
        char c = rel[i];
        u4[i] = (char)((c >= 'a' && c <= 'z') ? (c - 32) : c);
    }
    u4[i] = '\0';
    if (i < 4u || (rel[4] != '/' && rel[4] != '\\'))
        return 0;
    return (strcmp(u4, "CARD") == 0 || strcmp(u4, "NAND") == 0) ? 1 : 0;
}

static void moral_cbfs_copy_path_slashes(char *dst, size_t cap, const char *src, char to_sep)
{
    size_t k;

    if (dst == NULL || cap < 2u || src == NULL)
        return;
    for (k = 0; src[k] != '\0' && k + 1u < cap; k++)
        dst[k] = (src[k] == '/' || src[k] == '\\') ? to_sep : src[k];
    dst[k] = '\0';
}

static int moral_cbfs_needle_push(const char *arr[CBFS_NAND_NEEDLE_CAP], int *nn, const char *s)
{
    int j;
    size_t L;

    if (s == NULL || *nn >= CBFS_NAND_NEEDLE_CAP)
        return 0;
    L = strlen(s);
    if (L < 8u)
        return 0;
    for (j = 0; j < *nn; j++)
    {
        if (strcmp(arr[j], s) == 0)
            return 0;
    }
    arr[*nn] = s;
    (*nn)++;
    return 1;
}

/*
 * 固件虚拟路径可能是 CARD/.system/...，数据仍在 moral-i9.bin（NandFlashCard）里。
 * 无完整 CBFS 解析时：在镜像中搜 UTF-8 路径/文件名，再在附近取合法 BMP 整文件。
 */
static FILE *moral_cbfs_fopen_nand_scan(const char *rel_utf8, char *last_tried, size_t last_tried_sz,
                                        u32 *out_str_hits)
{
    const u8 *hay;
    size_t hl;
    char alt_bs[1024];
    char alt_sl[1024];
    char pref[7][1100];
    const char *needles[CBFS_NAND_NEEDLE_CAP];
    int nn;
    int ni;
    size_t nl;
    const u8 *hit;
    const u8 *next;
    u32 bmp_off;
    u32 bmp_sz;
    u32 hit_count;
    u32 str_hits_total;
    FILE *tf;

    if (out_str_hits != NULL)
        *out_str_hits = 0u;
    if (rel_utf8 == NULL || g_nand_flash_image_bytes < 1024u)
        return NULL;
    hay = (const u8 *)NandFlashCard;
    hl = (size_t)g_nand_flash_image_bytes;

    nn = 0;
    moral_cbfs_copy_path_slashes(alt_bs, sizeof(alt_bs), rel_utf8, '\\');
    moral_cbfs_copy_path_slashes(alt_sl, sizeof(alt_sl), rel_utf8, '/');
    moral_cbfs_needle_push(needles, &nn, rel_utf8);
    if (strcmp(alt_bs, rel_utf8) != 0)
        moral_cbfs_needle_push(needles, &nn, alt_bs);
    if (strcmp(alt_sl, rel_utf8) != 0 && strcmp(alt_sl, alt_bs) != 0)
        moral_cbfs_needle_push(needles, &nn, alt_sl);

    if (!moral_cbfs_rel_has_vol_prefix(rel_utf8))
    {
        snprintf(pref[0], sizeof(pref[0]), "CARD\\%s", alt_bs);
        snprintf(pref[1], sizeof(pref[1]), "CARD/%s", alt_sl);
        snprintf(pref[2], sizeof(pref[2]), "NAND\\%s", alt_bs);
        snprintf(pref[3], sizeof(pref[3]), "NAND/%s", alt_sl);
        moral_cbfs_needle_push(needles, &nn, pref[0]);
        moral_cbfs_needle_push(needles, &nn, pref[1]);
        moral_cbfs_needle_push(needles, &nn, pref[2]);
        moral_cbfs_needle_push(needles, &nn, pref[3]);
        /* 镜像内常见前导 / 的 POSIX 路径，如 /CARD/.system/... */
        snprintf(pref[4], sizeof(pref[4]), "/CARD/%s", alt_sl);
        snprintf(pref[5], sizeof(pref[5]), "/NAND/%s", alt_sl);
        moral_cbfs_needle_push(needles, &nn, pref[4]);
        moral_cbfs_needle_push(needles, &nn, pref[5]);
    }
    else if (alt_sl[0] != '/')
    {
        snprintf(pref[6], sizeof(pref[6]), "/%s", alt_sl);
        moral_cbfs_needle_push(needles, &nn, pref[6]);
    }

    moral_cbfs_needle_push(needles, &nn, moral_cbfs_basename_utf8(rel_utf8));

    str_hits_total = 0u;
    for (ni = 0; ni < nn; ni++)
    {
        nl = strlen(needles[ni]);
        if (nl < 8u)
            continue;
        hit = hay;
        hit_count = 0u;
        while (hit_count < 400u && (next = moral_memmem(hit, hl - (size_t)(hit - hay), (const u8 *)needles[ni], nl)) != NULL)
        {
            hit_count++;
            str_hits_total++;
            if (moral_cbfs_find_bmp_near_path(hay, hl, (size_t)(next - hay), &bmp_off, &bmp_sz))
            {
                tf = tmpfile();
                if (tf == NULL)
                    return NULL;
                if (fwrite(hay + bmp_off, 1u, (size_t)bmp_sz, tf) != (size_t)bmp_sz)
                {
                    fclose(tf);
                    return NULL;
                }
                rewind(tf);
                if (last_tried != NULL && last_tried_sz > 0u)
                    snprintf(last_tried, last_tried_sz, "NandFlashCard BMP @0x%X len=%u (path~%s)", bmp_off, bmp_sz,
                             needles[ni]);
                return tf;
            }
            hit = next + 1u;
        }
    }
    if (out_str_hits != NULL)
        *out_str_hits = str_hits_total;
    return NULL;
}

#ifdef _WIN32
static void moral_cbfs_win_slashes(char *p)
{
    if (p == NULL)
        return;
    for (; *p != '\0'; p++)
    {
        if (*p == '/')
            *p = '\\';
    }
}

static void moral_cbfs_copy_abs_or_raw(char *dst, size_t dst_sz, const char *full)
{
    char tmp[1100];
    if (dst == NULL || dst_sz == 0u || full == NULL)
        return;
    if (_fullpath(tmp, full, sizeof(tmp)) != NULL)
        snprintf(dst, dst_sz, "%s", tmp);
    else
        snprintf(dst, dst_sz, "%s", full);
}
#else
static void moral_cbfs_win_slashes(char *p)
{
    (void)p;
}

static void moral_cbfs_copy_abs_or_raw(char *dst, size_t dst_sz, const char *full)
{
    if (dst != NULL && dst_sz > 0u && full != NULL)
        snprintf(dst, dst_sz, "%s", full);
}
#endif

/*
 * 与 open_sd_image_with_fallback 一致：SDL_GetBasePath() 多为 ...\proj\bin\。
 * 只能再拼 Rom\ 或 ..\Rom\，禁止 base+bin\Rom\（会变成 ...\bin\bin\Rom\）。
 * 无 base 时 cwd 常为工程根：先试 Rom\\，再试 bin\\Rom\\（资源常放在 bin\\Rom\\ 与 exe 同树）。
 * cwd=...\\bin 时 bin\\bin\\Rom\\ 会失败一次，可接受。
 */
static FILE *moral_cbfs_try_open_built_path(char *full, const char *fmode, char *last_tried, size_t last_tried_sz)
{
    FILE *fp;

    moral_cbfs_win_slashes(full);
    if (last_tried != NULL && last_tried_sz > 0u)
        moral_cbfs_copy_abs_or_raw(last_tried, last_tried_sz, full);
    fp = fopen(full, fmode);
    return fp;
}

static FILE *moral_cbfs_try_open_sub_rel(const char *sub, const char *rel_utf8, const char *fmode,
                                         char *last_tried, size_t last_tried_sz)
{
    static const char *const k_with_base[] = {
        "Rom\\",
        "..\\Rom\\",
    };
    /* 勿加 bin\\Rom\\：若 cwd 已是 ...\\bin 会变成 bin\\bin\\Rom\\ */
    static const char *const k_cwd_only[] = {
        "Rom\\",
        "..\\Rom\\",
        "bin\\Rom\\",
        "..\\bin\\Rom\\",
    };
    char full[1100];
    char *sdl_base;
    size_t i;
    FILE *fp;

    sdl_base = SDL_GetBasePath();
    if (sdl_base != NULL)
    {
        for (i = 0; i < sizeof(k_with_base) / sizeof(k_with_base[0]); i++)
        {
            snprintf(full, sizeof(full), "%s%s%s%s", sdl_base, k_with_base[i], sub, rel_utf8);
            fp = moral_cbfs_try_open_built_path(full, fmode, last_tried, last_tried_sz);
            if (fp != NULL)
            {
                SDL_free(sdl_base);
                return fp;
            }
        }
        SDL_free(sdl_base);
    }

    for (i = 0; i < sizeof(k_cwd_only) / sizeof(k_cwd_only[0]); i++)
    {
        snprintf(full, sizeof(full), "%s%s%s", k_cwd_only[i], sub, rel_utf8);
        fp = moral_cbfs_try_open_built_path(full, fmode, last_tried, last_tried_sz);
        if (fp != NULL)
            return fp;
    }
    return NULL;
}

static FILE *moral_cbfs_fopen_host(const char *rel_utf8, u32 drive, u32 open_mode_flags)
{
    const char *sub_first;
    const char *sub_second;
    const char *fmode;
    FILE *fp;
    char last[1024];
    u8 from_nand_img;
    u32 nand_str_hits = 0u;

    last[0] = '\0';
    from_nand_img = 0u;

    if (rel_utf8 == NULL || moral_path_has_dotdot(rel_utf8))
        return NULL;

    /*
     * drive 2：固件常报 CARD/.system/...，数据仍在 NAND；宿主目录优先 NAND 再 CARD。
     * drive 4：典型为卡内路径，先 CARD 再 NAND。
     */
    if (drive == 4u)
    {
        sub_first = MORAL_CBFS_HOST_SUBDIR_CARD;
        sub_second = MORAL_CBFS_HOST_SUBDIR_NAND;
    }
    else if (drive == 2u)
    {
        sub_first = MORAL_CBFS_HOST_SUBDIR_NAND;
        sub_second = MORAL_CBFS_HOST_SUBDIR_CARD;
    }
    else
        return NULL;

    if ((open_mode_flags & 0x202u) != 0u || (open_mode_flags & 0x10u) != 0u)
        fmode = "r+b";
    else
        fmode = "rb";

    fp = moral_cbfs_try_open_sub_rel(sub_first, rel_utf8, fmode, last, sizeof(last));
    if (fp == NULL)
        fp = moral_cbfs_try_open_sub_rel(sub_second, rel_utf8, fmode, last, sizeof(last));

    if (fp == NULL && fmode[0] == 'r' && fmode[1] == 'b' && fmode[2] == '+')
    {
        fp = moral_cbfs_try_open_sub_rel(sub_first, rel_utf8, "w+b", last, sizeof(last));
        if (fp == NULL)
            fp = moral_cbfs_try_open_sub_rel(sub_second, rel_utf8, "w+b", last, sizeof(last));
    }

    /*
     * 宿主无对应文件时从 NAND 缓冲扫路径+BMP。固件常以 r+b（如 mode 含 0x200）打开资源，
     * 若此处仍要求纯 rb 则永远不会走扫描。
     */
    if (fp == NULL)
    {
        fp = moral_cbfs_fopen_nand_scan(rel_utf8, last, sizeof(last), &nand_str_hits);
        if (fp != NULL)
            from_nand_img = 1u;
    }

    if (fp == NULL && MORAL_CBFS_HOST_LOG_MAX > 0u && g_cbfs_host_log_count < MORAL_CBFS_HOST_LOG_MAX)
    {
        static char s_miss_rel[1024];
        u8 is_same_again;

        is_same_again = (s_miss_rel[0] != '\0' && strcmp(s_miss_rel, rel_utf8) == 0) ? 1u : 0u;
        snprintf(s_miss_rel, sizeof(s_miss_rel), "%s", rel_utf8);
        if (is_same_again != 0u)
            goto cbfs_miss_skip_log;

        g_cbfs_host_log_count++;
        printf("[CBFS-host] fopen miss drive=%u mode=0x%x 固件路径~%s%s | 可放 Rom\\、bin\\Rom\\ 下 NAND\\ 或 CARD\\ 同相对路径；last: %s\n",
               (unsigned)drive, (unsigned)open_mode_flags,
               drive == 2u ? "CARD/" : "", rel_utf8, last[0] != '\0' ? last : "(none)");
        if (g_nand_flash_image_bytes < 1024u)
            printf("[CBFS-host] 说明: NAND镜像=%ubytes，过小或未加载，未扫描 NandFlashCard。\n",
                   (unsigned)g_nand_flash_image_bytes);
        else if (nand_str_hits == 0u)
            printf("[CBFS-host] 说明: NAND 镜像内 UTF-8 路径/basename 一次也未命中（与 BMP 无关）。"
                   "该资源多半不在 moral-i9.bin 明文里；请把真实 mms45.dat 放到上述目录，或从真卡/完整卡镜像取出。\n");
        else
            printf("[CBFS-host] 说明: NAND 镜像=%ubytes，路径类字符串命中约 %u 处，但附近±8KiB~+256KiB 无合法 BMP；"
                   "若 .dat 非 BMP 需放 Rom 或扩展解析。\n",
                   (unsigned)g_nand_flash_image_bytes, (unsigned)nand_str_hits);
    cbfs_miss_skip_log:
        ;
    }
    else if (fp != NULL && MORAL_CBFS_HOST_LOG_MAX > 0u && g_cbfs_host_log_count < MORAL_CBFS_HOST_LOG_MAX)
    {
        g_cbfs_host_log_count++;
        if (from_nand_img != 0u)
            printf("[CBFS-host] fopen ok drive=%u 来自 NAND 镜像 %s\n", (unsigned)drive, last);
        else
            printf("[CBFS-host] fopen ok drive=%u Rom\\%s%s\n", (unsigned)drive, sub_first, rel_utf8);
    }
    return fp;
}

static int moral_guest_file_pool_write_slot(int slot, u32 used, u32 ven_handle, u8 type_byte)
{
    u32 base;

    if (slot < 0 || slot >= CBFS_HOST_SLOTS || MORAL_CBFS_FILE_POOL_GVA == 0u)
        return -1;
    base = MORAL_CBFS_FILE_POOL_GVA + (u32)slot * 12u;
    if (uc_mem_write(MTK, base, &used, 4) != UC_ERR_OK)
        return -1;
    if (uc_mem_write(MTK, base + 4u, &ven_handle, 4) != UC_ERR_OK)
        return -1;
    if (uc_mem_write(MTK, base + 8u, &type_byte, 1) != UC_ERR_OK)
        return -1;
    return 0;
}

static int moral_guest_file_pool_read_used(int slot, u32 *used)
{
    u32 base;

    if (slot < 0 || slot >= CBFS_HOST_SLOTS || used == NULL || MORAL_CBFS_FILE_POOL_GVA == 0u)
        return -1;
    base = MORAL_CBFS_FILE_POOL_GVA + (u32)slot * 12u;
    return uc_mem_read(MTK, base, used, 4) == UC_ERR_OK ? 0 : -1;
}

static int moral_cbfs_find_free_slot(void)
{
    int i;
    u32 u;

    for (i = 0; i < CBFS_HOST_SLOTS; i++)
    {
        if (moral_guest_file_pool_read_used(i, &u) != 0)
            continue;
        if (u == 0u)
            return i;
    }
    return -1;
}

static void moral_thumb_return_r0(u32 r0)
{
    u32 lr;
    uc_reg_write(MTK, UC_ARM_REG_R0, &r0);
    uc_reg_read(MTK, UC_ARM_REG_LR, &lr);
    uc_reg_write(MTK, UC_ARM_REG_PC, &lr);
}

static int moral_cbfs_h_open(void)
{
    u32 drive, path_gva, mode;
    u16 wbuf[CBFS_PATH_UWCHAR_MAX];
    char utf8[1024];
    int si;
    FILE *fp;

    uc_reg_read(MTK, UC_ARM_REG_R0, &drive);
    uc_reg_read(MTK, UC_ARM_REG_R1, &path_gva);
    uc_reg_read(MTK, UC_ARM_REG_R2, &mode);

    if (drive != 2u && drive != 4u)
        return 0;

    if (path_gva < 0x1000u || path_gva >= 0x8000000u)
        return 0;

    if (moral_cbfs_read_guest_ucs2_z(path_gva, wbuf, CBFS_PATH_UWCHAR_MAX) < 0)
        return 0;
    if (moral_ucs2_to_utf8_path(wbuf, utf8, sizeof(utf8)) < 0)
        return 0;

    fp = moral_cbfs_fopen_host(utf8, drive, mode);
    if (fp == NULL)
        return 0;

    si = moral_cbfs_find_free_slot();
    if (si < 0)
    {
        fclose(fp);
        return 0;
    }

    g_cbfs_host_slots[si].fp = fp;
    g_cbfs_host_slots[si].in_use = 1u;
    /* type=2 与固件 NAND/CARD ven 槽一致；handle 置 0，读写在钩子内完成 */
    if (moral_guest_file_pool_write_slot(si, 1u, 0u, 2u) != 0)
    {
        fclose(fp);
        g_cbfs_host_slots[si].fp = NULL;
        g_cbfs_host_slots[si].in_use = 0u;
        return 0;
    }

    moral_thumb_return_r0((u32)si);
    return 1;
}

static int moral_cbfs_h_close(void)
{
    u32 slot;

    uc_reg_read(MTK, UC_ARM_REG_R0, &slot);
    if (slot >= (u32)CBFS_HOST_SLOTS)
        return 0;

    if (!g_cbfs_host_slots[slot].in_use || g_cbfs_host_slots[slot].fp == NULL)
        return 0;

    fclose(g_cbfs_host_slots[slot].fp);
    g_cbfs_host_slots[slot].fp = NULL;
    g_cbfs_host_slots[slot].in_use = 0u;
    moral_guest_file_pool_write_slot((int)slot, 0u, 0u, 0u);

    moral_thumb_return_r0(0u);
    return 1;
}

static int moral_cbfs_h_read(void)
{
    u32 buf_gva, len, slot;
    u32 used;
    size_t n;
    u8 stackbuf[4096];
    u8 *heapbuf = NULL;
    u8 *usebuf;
    size_t chunk;

    uc_reg_read(MTK, UC_ARM_REG_R0, &buf_gva);
    uc_reg_read(MTK, UC_ARM_REG_R1, &len);
    uc_reg_read(MTK, UC_ARM_REG_R2, &slot);

    if (slot >= (u32)CBFS_HOST_SLOTS || len > 0x100000u)
        return 0;
    if (moral_guest_file_pool_read_used((int)slot, &used) != 0 || used == 0u)
        return 0;
    if (!g_cbfs_host_slots[slot].in_use || g_cbfs_host_slots[slot].fp == NULL)
        return 0;

    chunk = (size_t)len;
    if (chunk <= sizeof(stackbuf))
        usebuf = stackbuf;
    else
    {
        heapbuf = (u8 *)SDL_malloc(chunk);
        if (heapbuf == NULL)
            return 0;
        usebuf = heapbuf;
    }

    n = fread(usebuf, 1u, chunk, g_cbfs_host_slots[slot].fp);
    if (n > 0u && buf_gva >= 0x1000u && buf_gva < 0x8000000u)
    {
        if (uc_mem_write(MTK, buf_gva, usebuf, (u32)n) != UC_ERR_OK)
            n = 0u;
    }

    if (heapbuf != NULL)
        SDL_free(heapbuf);

    moral_thumb_return_r0((u32)n);
    return 1;
}

static int moral_cbfs_h_write(void)
{
    u32 buf_gva, len, slot;
    u32 used;
    size_t n;
    u8 stackbuf[4096];
    u8 *heapbuf = NULL;
    u8 *usebuf;
    size_t chunk;

    uc_reg_read(MTK, UC_ARM_REG_R0, &buf_gva);
    uc_reg_read(MTK, UC_ARM_REG_R1, &len);
    uc_reg_read(MTK, UC_ARM_REG_R2, &slot);

    if (slot >= (u32)CBFS_HOST_SLOTS || len > 0x100000u)
        return 0;
    if (moral_guest_file_pool_read_used((int)slot, &used) != 0 || used == 0u)
        return 0;
    if (!g_cbfs_host_slots[slot].in_use || g_cbfs_host_slots[slot].fp == NULL)
        return 0;

    chunk = (size_t)len;
    if (chunk <= sizeof(stackbuf))
        usebuf = stackbuf;
    else
    {
        heapbuf = (u8 *)SDL_malloc(chunk);
        if (heapbuf == NULL)
            return 0;
        usebuf = heapbuf;
    }

    if (buf_gva >= 0x1000u && buf_gva < 0x8000000u &&
        uc_mem_read(MTK, buf_gva, usebuf, (u32)chunk) == UC_ERR_OK)
        n = fwrite(usebuf, 1u, chunk, g_cbfs_host_slots[slot].fp);
    else
        n = 0u;
    fflush(g_cbfs_host_slots[slot].fp);

    if (heapbuf != NULL)
        SDL_free(heapbuf);

    moral_thumb_return_r0((u32)n);
    return 1;
}

static int moral_cbfs_h_seek(void)
{
    u32 slot, off, whence;
    u32 used;
    int wh;
    long r;

    uc_reg_read(MTK, UC_ARM_REG_R0, &slot);
    uc_reg_read(MTK, UC_ARM_REG_R1, &off);
    uc_reg_read(MTK, UC_ARM_REG_R2, &whence);

    if (slot >= (u32)CBFS_HOST_SLOTS)
        return 0;
    if (moral_guest_file_pool_read_used((int)slot, &used) != 0 || used == 0u)
        return 0;
    if (!g_cbfs_host_slots[slot].in_use || g_cbfs_host_slots[slot].fp == NULL)
        return 0;

    wh = (whence == 0u) ? SEEK_SET : (whence == 1u) ? SEEK_CUR : SEEK_END;
    r = fseek(g_cbfs_host_slots[slot].fp, (long)off, wh);
    moral_thumb_return_r0((u32)r);
    return 1;
}

static int moral_cbfs_h_tell(void)
{
    u32 slot;
    u32 used;
    long pos;

    uc_reg_read(MTK, UC_ARM_REG_R0, &slot);
    if (slot >= (u32)CBFS_HOST_SLOTS)
        return 0;
    if (moral_guest_file_pool_read_used((int)slot, &used) != 0 || used == 0u)
        return 0;
    if (!g_cbfs_host_slots[slot].in_use || g_cbfs_host_slots[slot].fp == NULL)
        return 0;

    pos = ftell(g_cbfs_host_slots[slot].fp);
    if (pos < 0)
        pos = 0;
    moral_thumb_return_r0((u32)pos);
    return 1;
}

static int moral_cbfs_h_getfilesize(void)
{
    u32 slot;
    u32 used;
    long cur, end, beg;

    uc_reg_read(MTK, UC_ARM_REG_R0, &slot);
    if (slot >= (u32)CBFS_HOST_SLOTS)
        return 0;
    if (moral_guest_file_pool_read_used((int)slot, &used) != 0 || used == 0u)
        return 0;
    if (!g_cbfs_host_slots[slot].in_use || g_cbfs_host_slots[slot].fp == NULL)
        return 0;

    cur = ftell(g_cbfs_host_slots[slot].fp);
    if (cur < 0)
        cur = 0;
    fseek(g_cbfs_host_slots[slot].fp, 0L, SEEK_SET);
    beg = ftell(g_cbfs_host_slots[slot].fp);
    fseek(g_cbfs_host_slots[slot].fp, 0L, SEEK_END);
    end = ftell(g_cbfs_host_slots[slot].fp);
    fseek(g_cbfs_host_slots[slot].fp, cur, SEEK_SET);

    if (end < beg)
        moral_thumb_return_r0(0u);
    else
        moral_thumb_return_r0((u32)(end - beg));
    return 1;
}

int moral_cbfs_host_dispatch(u32 guest_pc)
{
    u32 apc;

#if !MORAL_CBFS_HOST_ENABLE
    return 0;
#else
    apc = guest_pc & ~1u;

    switch (apc)
    {
    case MORAL_CBFS_VM_FILE_OPEN_PC:
        return moral_cbfs_h_open();
    case MORAL_CBFS_VM_FILE_CLOSE_PC:
        return moral_cbfs_h_close();
    case MORAL_CBFS_VM_FILE_READ_PC:
        return moral_cbfs_h_read();
    case MORAL_CBFS_VM_FILE_WRITE_PC:
        return moral_cbfs_h_write();
    case MORAL_CBFS_VM_FILE_SEEK_PC:
        return moral_cbfs_h_seek();
    case MORAL_CBFS_VM_FILE_TELL_PC:
        return moral_cbfs_h_tell();
    case MORAL_CBFS_VM_FILE_GETSIZE_PC:
        return moral_cbfs_h_getfilesize();
    default:
        return 0;
    }
#endif
}
