#include "main.h"
#include "lcd.h"
#include "touchscreen.h"
#include <string.h>

/*
 * hookRam.c：Guest 对 MMIO 的读写在 Unicorn 里走 UC_HOOK_MEM_READ/WRITE，由此回调模拟硬件。
 * 含 SPI、定时器、显示引擎 DE、FCIE(NAND/SD)、UART、中断控制器、触摸 AUXADC、GPT、MSDC 等。
 * 与 main.c 的 UC_HOOK_CODE 分工：代码钩子改执行流/寄存器，本文件只伪造寄存器值与 DMA 数据搬运。
 *
 * DE 图层（完全对齐 IDA HalDispSetBufInfo）：
 *   Layer 0 (a1=0, OP主层) : 0x74003040(ptr_lo)/44(ptr_hi)/48(W)/4C(H)/50(pitch)
 *   Layer 1 (a1=1, PIP1)   : 0x74003054(ptr_lo)/58(ptr_hi)/5C(W)/60(H)/64(pitch)
 *   Layer 2 (a1=2, PIP2)   : 0x74003068(ptr_lo)/6C(ptr_hi)/70(W)/74(H)/78(pitch)
 *   Layer 3 (a1=3, PIP3)   : 0x74003080(ptr_lo)/84(ptr_hi)/88(W)/8C(H)/90(pitch)
 * PIP 位置：Layer1→0x740030A0..AC, Layer2→B0..BC, Layer3→C0..CC
 * Alpha   : 0x740030D0(L1)/D4(L2)/D8(L3)
 * ColorKey: 0x740030DC/E0/E4(L1), E8/EC/F0(L2), F4/F8/FC(L3); 控制字=0x74003100
 * 触发     : 0x7400313C(DE trigger), 0x74003140(FMark/Reset)
 * 脏矩形 w/h 仅来自 SWI 0xc166，不与图层寄存器混用。
 */

bool hookInsnInvalid(uc_engine *uc, void *user_data);
void hookRamCallBack(uc_engine *uc, uc_mem_type type, uint64_t address, uint32_t size, int64_t value, u32 data);
void handleLcdReg(uint64_t address, u32 data, uint64_t value);
void handleTouchScreenReg(uint64_t address, u32 data, uint64_t value);
void handleGptReg(uint64_t address, u32 data, uint64_t value);

/* NC read：2048B page 内连续 sector → DMA + CIFD spare；first+count≤4 */
static void nand_nc_read_page_sectors_contig(nandPage2048 *p, u32 dmaPtr, u32 firstSector, u32 sectorCount)
{
    u32 byteOff;
    u32 u;
    if (firstSector > 3u || sectorCount < 1u || sectorCount > 4u || firstSector + sectorCount > 4u)
        return;
    byteOff = firstSector * 512u;
    uc_mem_write(MTK, dmaPtr, p->pageBuff + byteOff, 512u * sectorCount);
    for (u = 0; u < 8u * sectorCount; u++)
        nandSpareBuff[u] = p->spareBuff[firstSector * 8u + u];
    uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandSpareBuff, 32u * sectorCount);
}

static void nand_nc_write_page_sectors_contig(nandPage2048 *p, u32 dmaPtr, u32 firstSector, u32 sectorCount)
{
    u32 byteOff;
    if (firstSector > 3u || sectorCount < 1u || sectorCount > 4u || firstSector + sectorCount > 4u)
        return;
    byteOff = firstSector * 512u;
    uc_mem_read(MTK, dmaPtr, p->pageBuff + byteOff, 512u * sectorCount);
}

static u8 nand_nc_try_decode_sector_offset(u32 sectorInPage, u32 *firstSector)
{
    if (firstSector == NULL)
        return 0;

    if (sectorInPage <= 3u)
    {
        *firstSector = sectorInPage;
        return 1;
    }

    if ((sectorInPage % 512u) == 0u)
    {
        u32 sectorIdx = sectorInPage / 512u;
        if (sectorIdx < 4u)
        {
            *firstSector = sectorIdx;
            return 1;
        }
    }

    if ((sectorInPage % 256u) == 0u)
    {
        u32 sectorIdx = sectorInPage / 256u;
        if (sectorIdx < 4u)
        {
            *firstSector = sectorIdx;
            return 1;
        }
    }

    return 0;
}

static u8 nand_nc_resolve_sector_window(u32 paramSect, u32 sectorInPage, u32 *firstSector, u32 *sectorCount)
{
    u32 fs = 0u;
    u32 ns = 0u;
    u32 low = paramSect & 0x7Fu;
    u32 high = paramSect >> 7;

    if (high <= 3u && low >= 1u && low <= 7u && (low & 1u) == 1u)
    {
        /* paramSect 高 2bit=起始 sector，低 7bit 奇数 1/3/5…=连续 sector 个数 */
        fs = high;
        ns = (low + 1u) / 2u;
    }
    else
    {
        nand_nc_try_decode_sector_offset(sectorInPage, &fs);
        if (paramSect >= 1u && paramSect <= 4u)
            ns = paramSect;
        else if ((paramSect & 0xFFu) >= 1u && (paramSect & 0xFFu) <= 4u)
            ns = paramSect & 0xFFu;
    }

    if (fs >= 4u)
        return 0;
    if (ns == 0u)
        ns = 1u;
    if (fs + ns > 4u)
        ns = 4u - fs;
    if (ns == 0u)
        return 0;

    if (firstSector != NULL)
        *firstSector = fs;
    if (sectorCount != NULL)
        *sectorCount = ns;
    return 1;
}

u32 halTimerCnt = 0;
u32 halTimerCntMax = 0;
u32 DrvTimerStdaTimerGetTick = 0;

u8 de_small_pending = 0;

static uint64_t cursor_de_last_draw = 0;

u8 SPI_CMD_Buf[256];
u32 SPI_CMD_Buf_Idx = 0;

u8 SPI_DATA_Buf[256];
int SPI_DATA_Buf_Idx = 0;

u8 UART1_Buf[1024];
u32 UART1_Buf_Idx = 0;

u8 UART2_Buf[1024];
u32 UART2_Buf_Idx = 0;

u8 SD_CMD_L;
u8 SD_CMD_H;
u32 SD_CMD_RSP_Buff[8];
u32 g_sd_dma_phys_addr = 0;

u32 SD_CMD_Buff[8];
u16 SD_CMD_Buff_Idx = 0;
u32 nandFlashCMD = 0;
u32 nandFlashRow = 0;

u32 nandFlashCmdIdx = 0;
static u8 nand_op_pending = 0;
u16 nandFlashCMDData[32];

u32 nandFlashSectorSize = 0;
u32 nandParamSect = 0;
u32 LCD_CMD_Data = 0;

static u32 auxadc_last_cmd = 0;

static u8 auxadc_log_count = 0;

static u32 auxadc_resolve_channel(u32 cmd)
{
    u32 i, j;
    u32 slices[6];
    static const u32 known[] = {0x711u, 0xB81u, 0x781u, 0x7B1u, 0x181u, 0x191u};
    slices[0] = cmd & 0xFFFu;
    slices[1] = (cmd >> 4) & 0xFFFu;
    slices[2] = (cmd >> 8) & 0xFFFu;
    slices[3] = (cmd >> 12) & 0xFFFu;
    slices[4] = (cmd >> 16) & 0xFFFu;
    slices[5] = cmd & 0xFFu;
    for (i = 0; i < 6u; i++)
    {
        for (j = 0; j < sizeof(known) / sizeof(known[0]); j++)
        {
            if (slices[i] == known[j])
                return known[j];
        }
    }
    return cmd & 0xFFFu;
}

static u32 auxadc_get_result(void)
{
    u32 cmd12 = auxadc_resolve_channel(auxadc_last_cmd);
    u32 result;
    switch (cmd12)
    {
    case 0x711:
        result = (touchX < 240u ? touchX : 239u) * 1023u / 240u;
        break;
    case 0xB81:
        result = (touchY < 400u ? touchY : 399u) * 1023u / 400u;
        break;
    case 0x781:
        result = isTouchDown ? 200u : 0u;
        break;
    case 0x7B1:
        result = isTouchDown ? 600u : 1023u;
        break;
    case 0x181:
        result = isTouchDown ? 300u : 0u;
        break;
    case 0x191:
        result = isTouchDown ? 300u : 0u;
        break;
    default:
        result = isTouchDown ? 400u : 0u;
        break;
    }
    if (MORAL_LOG_TOUCH_DEBUG && auxadc_log_count < 5)
    {
        auxadc_log_count++;
        printf("[ADC-read] raw_cmd=0x%x eff_ch=0x%x result=%u down=%d x=%u y=%u\n",
               auxadc_last_cmd, cmd12, result, isTouchDown, touchX, touchY);
    }
    return result;
}

#define DE_PANEL_W 240u
#define DE_PANEL_H 400u
#define DE_BPP 2u

static u8 de_blit_logged = 0;

static void de_blit_rgb565_to_surface(SDL_Surface *sfc, u8 *srcBuf, u16 dstX, u16 dstY, u16 w, u16 h, u32 cachePitch)
{
    if (!sfc || !srcBuf)
        return;

    u32 sfcBpp = (u32)sfc->format->BytesPerPixel;
    u32 sfcPitch = (u32)sfc->pitch;

    for (u16 yi = 0; yi < h && (dstY + yi) < (u16)sfc->h; yi++)
    {
        u16 *srcRow = (u16 *)(srcBuf + cachePitch * (u32)yi);
        u8 *dstRow = (u8 *)sfc->pixels + sfcPitch * (u32)(dstY + yi) + sfcBpp * (u32)dstX;
        u16 maxW = w;
        if (dstX + maxW > (u16)sfc->w)
            maxW = (u16)sfc->w - dstX;

        if (sfcBpp == 4)
        {
            u32 *dst32 = (u32 *)dstRow;
            for (u16 xi = 0; xi < maxW; xi++)
            {
                u16 c = srcRow[xi];
                u32 r = (c >> 11) & 0x1F;
                u32 g = (c >> 5) & 0x3F;
                u32 b = c & 0x1F;
                dst32[xi] = (0xFFu << 24) | ((r << 3 | r >> 2) << 16) | ((g << 2 | g >> 4) << 8) | (b << 3 | b >> 2);
            }
        }
        else if (sfcBpp == 2 && sfc->format->Rmask == 0xF800u && sfc->format->Gmask == 0x07E0u &&
                 sfc->format->Bmask == 0x001Fu)
        {
            memcpy(dstRow, srcRow, (size_t)maxW * 2u);
        }
        else
        {
            for (u16 xi = 0; xi < maxW; xi++)
            {
                u16 color = srcRow[xi];
                SDL_PutPixel32(sfc, dstX + xi, dstY + yi,
                               SDL_MapRGB(sfc->format, (u8)PIXEL565R(color), (u8)PIXEL565G(color), (u8)PIXEL565B(color)));
            }
        }
    }
}

static void de_blit_to_sdl(u16 dstX, u16 dstY, u16 w, u16 h, u32 cachePitch)
{
    SDL_Surface *sfc = SDL_GetWindowSurface(window);
    if (!sfc)
        return;

    if (!de_blit_logged)
    {
        de_blit_logged = 1;
#if MORAL_LOG_HOT_PATH
        printf("[SDL-surface] w=%d h=%d pitch=%d bpp=%d Rmask=0x%x\n",
               sfc->w, sfc->h, sfc->pitch, sfc->format->BytesPerPixel, sfc->format->Rmask);
#endif
    }

    /* 主线程读 Lcd_Cache_Buffer 时加锁，与 emu 线程写帧互斥，消除撕裂 */
    pthread_mutex_lock(&g_lcd_frame_mutex);
    de_blit_rgb565_to_surface(sfc, Lcd_Cache_Buffer, dstX, dstY, w, h, cachePitch);
    pthread_mutex_unlock(&g_lcd_frame_mutex);
}

/* HalDispSetBufInfo：单层 Guest 缓冲在 MIU 中的跨度（字节），用于判断 src 落在哪一层 */
static u32 de_layer_span_bytes(u32 pitch, u32 w, u32 h)
{
    u32 rowMin = w * DE_BPP;

    if (w == 0u || h == 0u || pitch < rowMin || pitch >= 8192u)
        return 0u;
    if (w > 4096u || h > 4096u)
        return 0u;
    return (h - 1u) * pitch + w * DE_BPP;
}

/* 返回某图层的 pitch；传入该层已知 pitch/W，无效则回退 DE_PANEL_W×BPP */
static u32 de_pitch_or_default(u32 pitch, u32 w)
{
    u32 rowMin = (w > 0u && w <= (u32)DE_PANEL_W) ? (u32)w * DE_BPP : (u32)DE_PANEL_W * DE_BPP;
    if (pitch >= rowMin && pitch < 8192u)
        return pitch;
    return rowMin;
}

/* 返回每个 Layer 对应的读行距（优先用已知 pitch） */
static u32 de_row_pitch_for_layer_base(u32 base)
{
    u32 row = (u32)DE_PANEL_W * DE_BPP;

    if (base == DE_Layer0_Ptr && DE_Layer0_W > 0u)
        return de_pitch_or_default(DE_Layer0_Pitch, DE_Layer0_W);
    if (base == DE_Layer1_Ptr && DE_Layer1_W > 0u)
        return de_pitch_or_default(DE_Layer1_Pitch, DE_Layer1_W);
    if (base == DE_Layer2_Ptr && DE_Layer2_W > 0u)
        return de_pitch_or_default(DE_Layer2_Pitch, DE_Layer2_W);
    if (base == DE_Layer3_Ptr && DE_Layer3_W > 0u)
        return de_pitch_or_default(DE_Layer3_Pitch, DE_Layer3_W);
    return row;
}

/* 全屏读时的 pitch：DE_Layer0_Pitch 始终是整屏行距（固件设 2*GetScreenWidth()=480） */
static u32 de_guest_pitch_for_full_panel_read(u32 expandBase)
{
    (void)expandBase;
    u32 p = DE_Layer0_Pitch;
    u32 rowMin = (u32)DE_PANEL_W * DE_BPP;
    if (p >= rowMin && p < 8192u)
        return p;
    return rowMin;
}

/* 周期拉帧 / 简易满屏读：必须用 Guest 真实行距，勿硬编码 240×2 */
static u32 de_panel_read_pitch_from_base(u32 srcBase)
{
    u32 rowMin = (u32)DE_PANEL_W * DE_BPP;
    u32 p = de_guest_pitch_for_full_panel_read(srcBase);
    if (p < rowMin || p >= 8192u)
        p = rowMin;
    return p;
}

/*
 * 检查 srcBuf 是否落在某个已知帧缓冲区间内，返回该帧缓冲基址。
 * 固件使用双缓冲（DispSetLayerFormat 交替配不同 GDI buffer），
 * 用 Lcd_FullScreen_Ptr / Lcd_FullScreen_Ptr2 跟踪两个基址。
 */
static u32 de_guest_fb_base_containing(u32 srcBuf)
{
    u32 fbBytes = (u32)DE_PANEL_W * (u32)DE_PANEL_H * DE_BPP;

    if (Lcd_FullScreen_Ptr >= 0x1000u && Lcd_FullScreen_Ptr < 0x8000000u &&
        srcBuf >= Lcd_FullScreen_Ptr && srcBuf < Lcd_FullScreen_Ptr + fbBytes)
        return Lcd_FullScreen_Ptr;

    if (Lcd_FullScreen_Ptr2 >= 0x1000u && Lcd_FullScreen_Ptr2 < 0x8000000u &&
        srcBuf >= Lcd_FullScreen_Ptr2 && srcBuf < Lcd_FullScreen_Ptr2 + fbBytes)
        return Lcd_FullScreen_Ptr2;

    return 0u;
}

/*
 * srcBuf 是帧缓冲基址 + 脏区偏移（y*pitch + x*BPP）。
 * 用 Lcd_FullScreen_Ptr（帧缓冲基址）和 pitch=480 反推脏区的 (x, y)。
 */
static void de_resolve_blit_dest(u32 srcBuf, u16 *dstX, u16 *dstY)
{
    u32 pitch = (DE_Layer0_Pitch >= (u32)DE_PANEL_W * DE_BPP && DE_Layer0_Pitch < 8192u)
                ? DE_Layer0_Pitch : (u32)DE_PANEL_W * DE_BPP;

    if (Lcd_FullScreen_Ptr >= 0x1000u && srcBuf >= Lcd_FullScreen_Ptr)
    {
        u32 off = srcBuf - Lcd_FullScreen_Ptr;
        u32 ty = off / pitch;
        u32 tx = (off % pitch) / DE_BPP;
        if (ty < DE_PANEL_H && tx < DE_PANEL_W)
        {
            *dstX = (u16)tx;
            *dstY = (u16)ty;
            return;
        }
    }
    *dstX = 0;
    *dstY = 0;
}

static int de_read_fb(u32 srcAddr, u32 guestPitch, u16 w, u16 h)
{
    u32 rowBytes = (u32)w * DE_BPP;
    if (rowBytes == 0 || h == 0)
        return -1;
    if ((u32)h * rowBytes > sizeof(Lcd_Cache_Buffer))
        return -1;

    /* 锁住帧缓冲区间：防止主线程读到写了一半的帧（撕裂） */
    pthread_mutex_lock(&g_lcd_frame_mutex);
    int ret = 0;
    if (guestPitch == rowBytes)
    {
        if (uc_mem_read(MTK, srcAddr, Lcd_Cache_Buffer, rowBytes * h) != UC_ERR_OK)
            ret = -1;
    }
    else
    {
        for (u16 y = 0; y < h; y++)
        {
            u32 readW = (guestPitch < rowBytes) ? guestPitch : rowBytes;
            u8 *dst = Lcd_Cache_Buffer + (u32)y * rowBytes;
            if (uc_mem_read(MTK, srcAddr + (u32)y * guestPitch, dst, readW) != UC_ERR_OK)
            {
                ret = -1;
                break;
            }
            if (readW < rowBytes)
                memset(dst + readW, 0, rowBytes - readW);
        }
    }
    pthread_mutex_unlock(&g_lcd_frame_mutex);
    return ret;
}

/*
 * 按面板行距 dstPitch 写入 RGB565：每行仅拷 srcW 个像素，右侧填 0；超出 srcH 的行保持 0。
 * 与 SDL/de_merge 使用的 240×2 行距一致，避免主层 W<H 与 pitch 和整屏 de_read_fb 混用造成条带花屏。
 */
static int de_read_guest_layer_to_panel(u8 *dst, u32 dstPitch, u16 panelW, u16 panelH, u32 srcAddr,
                                        u32 srcPitch, u16 srcW, u16 srcH)
{
    u32 rowDst = (u32)panelW * DE_BPP;
    u32 copyW;
    u32 rowCopyBytes;
    u16 y;

    if (srcAddr < 0x1000u || srcAddr >= 0x8000000u || dstPitch < rowDst)
        return -1;

    copyW = (u32)srcW;
    if (copyW > (u32)panelW)
        copyW = (u32)panelW;
    rowCopyBytes = copyW * DE_BPP;
    if (srcPitch < rowCopyBytes || srcPitch >= 8192u)
        srcPitch = rowCopyBytes;
    if (srcH > panelH)
        srcH = panelH;

    memset(dst, 0, (size_t)panelH * dstPitch);

    for (y = 0; y < srcH; y++)
    {
        u8 *rowd = dst + (u32)y * dstPitch;

        if (uc_mem_read(MTK, srcAddr + (u32)y * srcPitch, rowd, rowCopyBytes) != UC_ERR_OK)
            return -1;
        if (rowCopyBytes < rowDst)
            memset(rowd + rowCopyBytes, 0, rowDst - rowCopyBytes);
    }
    return 0;
}

/* HalDispSetAlpha 写入值 → 0..255 不透明度（255=完全不透明）；无法解析时偏不透明减少花洞 */
static unsigned de_alpha_u8_from_hw(short v)
{
    u16 u = (u16)(unsigned short)v;

    if (v == 0)
        return 255u;
    if (u == 64u)
        return 255u;
    if ((u & 0xFFu) == 176u)
    {
        unsigned hi = (unsigned)(u >> 8) & 0xFFu;
        return hi ? hi : 255u;
    }
    return 255u;
}

static u16 de_blend565_over(u16 s, u16 d, unsigned a)
{
    unsigned sr, sg, sb, dr, dg, db, inv;

    if (a >= 255u)
        return s;
    if (a == 0u)
        return d;
    sr = (unsigned)(s >> 11) & 0x1Fu;
    sg = (unsigned)(s >> 5) & 0x3Fu;
    sb = (unsigned)s & 0x1Fu;
    dr = (unsigned)(d >> 11) & 0x1Fu;
    dg = (unsigned)(d >> 5) & 0x3Fu;
    db = (unsigned)d & 0x1Fu;
    inv = 255u - a;
    sr = (sr * a + dr * inv + 127u) / 255u;
    sg = (sg * a + dg * inv + 127u) / 255u;
    sb = (sb * a + db * inv + 127u) / 255u;
    if (sr > 0x1Fu)
        sr = 0x1Fu;
    if (sg > 0x3Fu)
        sg = 0x3Fu;
    if (sb > 0x1Fu)
        sb = 0x1Fu;
    return (u16)((sr << 11) | (sg << 5) | sb);
}

/* mask!=0 时 (px^ref)&mask==0 视为透明（与常见 RGB565 colorkey 一致） */
static int de_ckey_skip_pixel(u16 px, const u16 ckey[3])
{
#if !MORAL_DE_BLEND_COLORKEY
    (void)px;
    (void)ckey;
    return 0;
#else
    u16 m = ckey[1];

    if (m == 0u)
        return 0;
    return ((px ^ ckey[0]) & m) == 0u;
#endif
}

/*
 * 子层 RGB565 按 PIP 压到 panel；可选色键/Alpha（HalDispSetColorKey / HalDispSetAlpha 镜像）。
 * 对 W/H/PIP 做夹紧，避免寄存器垃圾值导致 uc_mem_read 越界花屏。
 */
static void de_merge_layer_rgb565_into(u8 *panel, u32 panelPitch, u32 srcAddr, u32 srcPitch, u16 srcW,
                                       u16 srcH, const u16 pip[4], const u16 ckey[3], short alphaReg)
{
    u16 pipX = pip[0];
    u16 pipY = pip[1];
    u16 boxW = pip[2];
    u16 boxH = pip[3];
    u16 useW;
    u16 useH;
    u16 sw;
    u16 sh;
    u32 spanBytes;
    u8 rowbuf[240 * 2];
    unsigned alpha_u8;
    int need_pixel_blend;
    u16 yi;

    if (srcAddr < 0x1000u || srcAddr >= 0x8000000u || srcW == 0 || srcH == 0)
        return;

    sw = srcW;
    sh = srcH;
    if (sw > 1024u)
        sw = 1024u;
    if (sh > 1024u)
        sh = 1024u;

    if (srcPitch < (u32)sw * DE_BPP)
        srcPitch = (u32)sw * DE_BPP;

    spanBytes = de_layer_span_bytes(srcPitch, sw, sh);
    if (spanBytes == 0u)
        spanBytes = srcPitch * (u32)sh;

    if (pipX >= DE_PANEL_W || pipY >= DE_PANEL_H)
        return;

    useW = boxW;
    useH = boxH;
    if (boxW == 0u && boxH == 0u)
    {
        useW = sw > DE_PANEL_W ? (u16)DE_PANEL_W : sw;
        useH = sh > DE_PANEL_H ? (u16)DE_PANEL_H : sh;
        pipX = 0;
        pipY = 0;
    }
    else
    {
        if (useW == 0u || useW > sw)
            useW = sw;
        if (useH == 0u || useH > sh)
            useH = sh;
        if (useW > DE_PANEL_W)
            useW = (u16)DE_PANEL_W;
        if (useH > DE_PANEL_H)
            useH = (u16)DE_PANEL_H;
    }

#if MORAL_DE_BLEND_ALPHA
    alpha_u8 = de_alpha_u8_from_hw(alphaReg);
#else
    alpha_u8 = 255u;
#endif
    need_pixel_blend = 0;
#if MORAL_DE_BLEND_COLORKEY
    if (ckey[1] != 0u)
        need_pixel_blend = 1;
#endif
#if MORAL_DE_BLEND_ALPHA
    if (alpha_u8 < 255u)
        need_pixel_blend = 1;
#endif

    for (yi = 0; yi < useH; yi++)
    {
        u16 py = pipY + yi;
        u16 lineCopy;
        u32 rowOff = (u32)yi * srcPitch;

        if (py >= DE_PANEL_H)
            break;
        if (pipX >= DE_PANEL_W)
            continue;
        if (rowOff >= spanBytes)
            break;

        lineCopy = useW;
        if (lineCopy > sw)
            lineCopy = sw;
        if (spanBytes > rowOff)
        {
            u32 maxAtRow = (spanBytes - rowOff) / DE_BPP;

            if (maxAtRow == 0u)
                break;
            if (lineCopy > maxAtRow)
                lineCopy = (u16)maxAtRow;
        }
        else
            break;

        if (pipX + lineCopy > DE_PANEL_W)
            lineCopy = (u16)(DE_PANEL_W - pipX);

        if (uc_mem_read(MTK, srcAddr + rowOff, rowbuf, (u32)lineCopy * DE_BPP) != UC_ERR_OK)
            continue;

        if (!need_pixel_blend)
        {
            memcpy(panel + (u32)py * panelPitch + (u32)pipX * DE_BPP, rowbuf, (size_t)lineCopy * DE_BPP);
            continue;
        }

        {
            u16 xi;
            u8 *pdst = panel + (u32)py * panelPitch + (u32)pipX * DE_BPP;

            for (xi = 0; xi < lineCopy; xi++)
            {
                u16 s = (u16)((u16)rowbuf[(u32)xi * 2u] | ((u16)rowbuf[(u32)xi * 2u + 1u] << 8));
                u16 d = (u16)((u16)pdst[(u32)xi * 2u] | ((u16)pdst[(u32)xi * 2u + 1u] << 8));
                u16 out;

                if (de_ckey_skip_pixel(s, ckey))
                    continue;
#if MORAL_DE_BLEND_ALPHA
                out = de_blend565_over(s, d, alpha_u8);
#else
                out = s;
#endif
                pdst[(u32)xi * 2u] = (u8)(out & 0xFFu);
                pdst[(u32)xi * 2u + 1u] = (u8)(out >> 8);
            }
        }
    }
}

#if MORAL_DE_LAYER_COMPOSITE
static int de_layer_geo_ok(u32 w, u32 h)
{
    return w >= 1u && w <= 1024u && h >= 1u && h <= 1024u;
}

/* 判断某层是否是 PIP 子层（W×H 明显小于主层，非全屏） */
static int de_is_pip_sized(u32 w, u32 h)
{
    /* 全屏或接近全屏（>= 主层 80% 面积）视为主层备份，不叠加 */
    if (w >= (u32)(DE_PANEL_W - 20u) && h >= (u32)(DE_PANEL_H - 80u))
        return 0;
    return de_layer_geo_ok(w, h);
}

static int de_multilayer_composite_needed(void)
{
    /* PIP 子层：地址有效、不与主层同址、且尺寸明显小于全屏（非备用主层） */
    if (DE_Layer1_Ptr >= 0x1000u && DE_Layer1_Ptr != DE_Layer0_Ptr && de_is_pip_sized(DE_Layer1_W, DE_Layer1_H))
        return 1;
    if (DE_Layer2_Ptr >= 0x1000u && DE_Layer2_Ptr != DE_Layer0_Ptr && de_is_pip_sized(DE_Layer2_W, DE_Layer2_H))
        return 1;
    if (DE_Layer3_Ptr >= 0x1000u && DE_Layer3_Ptr != DE_Layer0_Ptr && de_is_pip_sized(DE_Layer3_W, DE_Layer3_H))
        return 1;
    return 0;
}

static int de_read_fb_composite_into(u8 *dst, u32 dstCap)
{
    u32 row = (u32)DE_PANEL_W * DE_BPP;
    u32 mainBase, mainPitch;
    u16 gw, gh;

    /* 主层优先 DE_Layer0（OP），若无效则用 Lcd_FullScreen_Ptr 做单层读 */
    if (DE_Layer0_Ptr >= 0x1000u && DE_Layer0_Ptr < 0x8000000u)
        mainBase = DE_Layer0_Ptr;
    else if (Lcd_FullScreen_Ptr >= 0x1000u && Lcd_FullScreen_Ptr < 0x8000000u)
        mainBase = Lcd_FullScreen_Ptr;
    else
        return -1;

    if ((u32)DE_PANEL_H * row > dstCap)
        return -1;

    gw = (DE_Layer0_W > 0u && DE_Layer0_W <= (u32)DE_PANEL_W) ? (u16)DE_Layer0_W : (u16)DE_PANEL_W;
    gh = (DE_Layer0_H > 0u && DE_Layer0_H <= (u32)DE_PANEL_H) ? (u16)DE_Layer0_H : (u16)DE_PANEL_H;
    mainPitch = de_pitch_or_default(DE_Layer0_Pitch, gw);

    if (de_read_guest_layer_to_panel(dst, row, DE_PANEL_W, DE_PANEL_H, mainBase, mainPitch, gw, gh) != 0)
        return -1;

    /* PIP Layer1 (IDA a1=1)：仅在明显是 PIP 小层时叠加 */
    if (DE_Layer1_Ptr >= 0x1000u && DE_Layer1_Ptr != mainBase && de_is_pip_sized(DE_Layer1_W, DE_Layer1_H))
        de_merge_layer_rgb565_into(dst, row, DE_Layer1_Ptr,
                                   de_pitch_or_default(DE_Layer1_Pitch, DE_Layer1_W),
                                   (u16)DE_Layer1_W, (u16)DE_Layer1_H,
                                   DE_PipLayer1, DE_CKey1, DE_AlphaHw[0]);
    /* PIP Layer2 (IDA a1=2) */
    if (DE_Layer2_Ptr >= 0x1000u && DE_Layer2_Ptr != mainBase &&
        DE_Layer2_Ptr != DE_Layer1_Ptr && de_is_pip_sized(DE_Layer2_W, DE_Layer2_H))
        de_merge_layer_rgb565_into(dst, row, DE_Layer2_Ptr,
                                   de_pitch_or_default(DE_Layer2_Pitch, DE_Layer2_W),
                                   (u16)DE_Layer2_W, (u16)DE_Layer2_H,
                                   DE_PipLayer2, DE_CKey2, DE_AlphaHw[1]);
    /* PIP Layer3 (IDA a1=3) */
    if (DE_Layer3_Ptr >= 0x1000u && DE_Layer3_Ptr != mainBase &&
        DE_Layer3_Ptr != DE_Layer1_Ptr && DE_Layer3_Ptr != DE_Layer2_Ptr &&
        de_is_pip_sized(DE_Layer3_W, DE_Layer3_H))
        de_merge_layer_rgb565_into(dst, row, DE_Layer3_Ptr,
                                   de_pitch_or_default(DE_Layer3_Pitch, DE_Layer3_W),
                                   (u16)DE_Layer3_W, (u16)DE_Layer3_H,
                                   DE_PipLayer3, DE_CKey3, DE_AlphaHw[2]);
    return 0;
}

static int de_read_fb_composite_full(void)
{
    /* 写 Lcd_Cache_Buffer，需与主线程读（de_blit_to_sdl）互斥 */
    pthread_mutex_lock(&g_lcd_frame_mutex);
    int r = de_read_fb_composite_into(Lcd_Cache_Buffer, sizeof(Lcd_Cache_Buffer));
    pthread_mutex_unlock(&g_lcd_frame_mutex);
    return r;
}
#endif /* MORAL_DE_LAYER_COMPOSITE */

static u32 de_periodic_call_cnt = 0;

#if !MORAL_EMU_DEDICATED_THREAD
static int de_read_fb_to(u8 *dstBuf, u32 dstBufSize, u32 srcAddr, u32 guestPitch, u16 w, u16 h)
{
    u32 rowBytes = (u32)w * DE_BPP;
    if (rowBytes == 0 || h == 0)
        return -1;
    if ((u32)h * rowBytes > dstBufSize)
        return -1;

    if (guestPitch == rowBytes)
    {
        if (uc_mem_read(MTK, srcAddr, dstBuf, rowBytes * h) != UC_ERR_OK)
            return -1;
    }
    else
    {
        for (u16 y = 0; y < h; y++)
        {
            u32 readW = (guestPitch < rowBytes) ? guestPitch : rowBytes;
            u8 *dst = dstBuf + (u32)y * rowBytes;
            if (uc_mem_read(MTK, srcAddr + (u32)y * guestPitch, dst, readW) != UC_ERR_OK)
                return -1;
            if (readW < rowBytes)
                memset(dst + readW, 0, rowBytes - readW);
        }
    }
    return 0;
}

static void de_blit_from(u8 *srcCache, u16 dstX, u16 dstY, u16 w, u16 h, u32 cachePitch)
{
    SDL_Surface *sfc = SDL_GetWindowSurface(window);
    if (!sfc)
        return;
    de_blit_rgb565_to_surface(sfc, srcCache, dstX, dstY, w, h, cachePitch);
}
#endif /* !MORAL_EMU_DEDICATED_THREAD */

static u8 de_deferred_pending = 0;
static u32 de_deferred_src_buf = 0;
static u32 de_deferred_pitch = 0;
static u16 de_deferred_w = 0;
static u16 de_deferred_h = 0;
static uint64_t de_deferred_last_draw = 0;
static uint64_t lcd_irq_last_enqueue = 0;

/* 主线程只置位 + uc_emu_stop；模拟线程停住 CPU 后再 uc_mem_read，避免与 Guest 写帧缓冲撕裂 */
#if MORAL_EMU_DEDICATED_THREAD
volatile int de_guest_fb_pull_requested;
volatile int de_periodic_fb_pull_requested;
#endif

static u8 s_lcd_host_blit_pending;
static u16 s_lcd_host_blit_dstX;
static u16 s_lcd_host_blit_dstY;
static u16 s_lcd_host_blit_w;
static u16 s_lcd_host_blit_h;
static u32 s_lcd_host_blit_pitch;

static void lcd_irq_enqueue_throttled(void)
{
    uint64_t now = moral_get_ticks_ms();
    uint64_t min_interval = 1000 / 120;

    if (min_interval < 1)
        min_interval = 1;
    if (lcd_irq_last_enqueue != 0 &&
        (now - lcd_irq_last_enqueue) < min_interval)
        return;

    lcd_irq_last_enqueue = now;
    EnqueueVMEvent(VM_EVENT_LCD_IRQ, 0, 0);
}

/*
 * 在 Guest 写 DE 触发寄存器 0x7400313C 的 MMIO 回调里同步执行（CPU 尚未执行下一条指令）：
 * 相当于「驱动提交本帧」后再采样显存，避免主线程轮询 uc_mem_read 与 CPU 写屏交错导致撕裂。
 * 仅把 RGB565 拷到 Lcd_Cache_Buffer 并登记 s_lcd_host_blit_*；SDL 画屏在主线程 de_lcd_apply_pending_host_blit。
 */
/*
 * IDA 调用链（每次 DE 刷新必经）：
 *   DrvLcdUpdate(lcd, x, y, w, h, layerFormat)
 *     → DrvLcdSetDisplayRange(x, y, w, h)      ← SWI 0xc166, 我们存到 Lcd_Update_X/Y/W/H
 *     → HalDispUpdateQuick → HalDispSetLayerBuffer
 *       → HalDispSetBufInfo(0, w, h, pitch=480, bufPtr + y*480 + x*2)
 *         写 0x74003040 = bufPtr + y*pitch + x*BPP  (脏区首地址)
 *         写 0x74003048 = w, 4C = h, 50 = pitch
 *     → HalDispForceUpdate → 写 0x7400313C = 1
 *
 * 因此 DE trigger 时：
 *   Lcd_Update_X/Y = 脏区屏幕坐标 (x, y)（由 SWI 0xc166 提供，100% 可靠）
 *   DE_Layer0_Ptr   = 帧缓冲基址 + y*pitch + x*BPP
 *   DE_Layer0_W/H   = 脏区宽高
 *   DE_Layer0_Pitch = 整屏行距（始终 480）
 *
 * 帧缓冲基址 = DE_Layer0_Ptr - Lcd_Update_Y * pitch - Lcd_Update_X * BPP
 */
static void de_try_snapshot_fb_at_de_trigger(u32 srcBuf_in, u32 pitch_in, u16 w_in, u16 h_in, int is_cursor)
{
    u32 srcBuf = srcBuf_in;
    u32 pitch = pitch_in;
    u16 w = w_in;
    u16 h = h_in;
    uint64_t now = moral_get_ticks_ms();
    u16 dstX = 0, dstY = 0;

    if (!is_cursor)
    {
        De_LastTriggerTime = now;
        De_PeriodicRefreshAllowed = 1;

        /*
         * 利用 Lcd_Update_X/Y（SWI 0xc166）推算帧缓冲基址，
         * 然后扩展为整屏读（把整帧拷到 Lcd_Cache_Buffer，保持 SDL 内容完整）。
         *
         * IDA HalDispSetLayerBuffer：RGB565 下 srcX 向下取偶对齐（v10 = 2*(srcX>>1)），
         * 再用对齐后的值算 offset = srcY * pitch + BPP * aligned_srcX。
         * Lcd_Update_X 是对齐前的原始值，这里做同样的对齐。
         */
        u32 updateX = Lcd_Update_X & ~1u;  /* RGB565 偶数像素对齐，与 IDA 一致 */
        u32 updateY = Lcd_Update_Y;
        u32 byteOff = updateY * pitch + updateX * DE_BPP;

        /* 反推帧缓冲基址 */
        u32 frameBase = 0u;
        if (byteOff <= srcBuf && srcBuf - byteOff >= 0x1000u)
            frameBase = srcBuf - byteOff;

        /* 记录已知帧缓冲基址（双缓冲跟踪） */
        if (frameBase >= 0x1000u && frameBase < 0x8000000u)
        {
            if (frameBase != Lcd_FullScreen_Ptr && frameBase != Lcd_FullScreen_Ptr2)
            {
                Lcd_FullScreen_Ptr2 = Lcd_FullScreen_Ptr;
                Lcd_FullScreen_Ptr = frameBase;
            }
            else if (frameBase != Lcd_FullScreen_Ptr)
            {
                /* 已知的第二个 base 又被使用了，提升为 primary */
                Lcd_FullScreen_Ptr2 = Lcd_FullScreen_Ptr;
                Lcd_FullScreen_Ptr = frameBase;
            }
        }

        /* 扩展为整屏读 */
        if (frameBase >= 0x1000u && frameBase < 0x8000000u)
        {
            srcBuf = frameBase;
            w = (u16)DE_PANEL_W;
            h = (u16)DE_PANEL_H;
            /* pitch 保持 DE_Layer0_Pitch（整屏行距） */
        }
        else if (w < (DE_PANEL_W - 20u) || h < (DE_PANEL_H - 80u))
        {
            /* 无法推算基址，兜底用 Lcd_FullScreen_Ptr */
            u32 expandBase = de_guest_fb_base_containing(srcBuf_in);
            if (expandBase != 0u)
            {
                srcBuf = expandBase;
                pitch = de_guest_pitch_for_full_panel_read(expandBase);
                w = (u16)DE_PANEL_W;
                h = (u16)DE_PANEL_H;
            }
            else
            {
                /* 完全无法扩展，只画局部——用 Lcd_Update_X/Y 做目的坐标 */
                dstX = (u16)updateX;
                dstY = (u16)updateY;
            }
        }
    }

    if (srcBuf < 0x1000u || srcBuf >= 0x8000000u || w == 0 || h == 0)
        return;

    if (pitch == 0u)
        pitch = (u32)w * DE_BPP;

#if MORAL_DE_LAYER_COMPOSITE
    {
        int read_ok = 0;

        if (!is_cursor && w >= DE_PANEL_W - 20u && h >= DE_PANEL_H - 80u && de_multilayer_composite_needed())
        {
            read_ok = (de_read_fb_composite_full() == 0);
            if (!read_ok)
                read_ok = (de_read_fb(srcBuf, pitch, w, h) == 0);
        }
        else
            read_ok = (de_read_fb(srcBuf, pitch, w, h) == 0);
        if (!read_ok)
            return;
    }
#else
    if (de_read_fb(srcBuf, pitch, w, h) != 0)
        return;
#endif

    if (!is_cursor && w >= DE_PANEL_W - 20u && h >= DE_PANEL_H - 80u)
    {
        u32 rowBytes = (u32)w * DE_BPP;
        u32 step = (rowBytes < 16u) ? 2u : 16u;
        u32 cnt = 0, white = 0;
        for (u32 off = 0; off < (u32)h * rowBytes && cnt < 64u; off += step)
        {
            cnt++;
            if (off + 1u < (u32)h * rowBytes &&
                Lcd_Cache_Buffer[off] == 0xFFu && Lcd_Cache_Buffer[off + 1u] == 0xFFu)
                white++;
        }
        if (cnt > 0 && white * 10u >= cnt * 9u)
            return;
    }

    /* 整屏扩展成功时 dstX/dstY 保持 0；局部回退时已设为 Lcd_Update_X/Y */
    if (w == DE_PANEL_W && h == DE_PANEL_H)
    {
        dstX = 0;
        dstY = 0;
    }

    s_lcd_host_blit_dstX = dstX;
    s_lcd_host_blit_dstY = dstY;
    s_lcd_host_blit_w = w;
    s_lcd_host_blit_h = h;
    s_lcd_host_blit_pitch = (u32)w * DE_BPP;
    s_lcd_host_blit_pending = 1;
    if (!is_cursor)
        de_deferred_last_draw = now;
    Perf_LcdRefreshCount++;
}

void de_emulator_flush_pending(void)
{
    u32 srcBuf;
    u32 pitch;
    u16 w;
    u16 h;

    if (!de_deferred_pending)
        return;

#if MORAL_DE_FLUSH_MIN_INTERVAL_MS > 0
    {
        uint64_t min_iv = (uint64_t)MORAL_DE_FLUSH_MIN_INTERVAL_MS;

        if (min_iv < 1)
            min_iv = 1;
        uint64_t now = moral_get_ticks_ms();
        if (de_deferred_last_draw != 0 && (now - de_deferred_last_draw) < min_iv)
            return; /* 保留 de_deferred_pending，下轮再读 FB；勿丢弃刷新请求 */
    }
#endif

#if !MORAL_EMU_DEDICATED_THREAD
    uint64_t now = moral_get_ticks_ms();
#endif

    srcBuf = de_deferred_src_buf;
    pitch = de_deferred_pitch;
    w = de_deferred_w;
    h = de_deferred_h;

    if (w < (DE_PANEL_W - 20u) || h < (DE_PANEL_H - 80u))
    {
        u32 expandBase = de_guest_fb_base_containing(de_deferred_src_buf);
        if (expandBase != 0u)
        {
            srcBuf = expandBase;
            pitch = de_guest_pitch_for_full_panel_read(expandBase);
            w = (u16)DE_PANEL_W;
            h = (u16)DE_PANEL_H;
        }
    }

    if (srcBuf < 0x1000u || srcBuf >= 0x8000000u || w == 0 || h == 0)
    {
        de_deferred_pending = 0;
        return;
    }
    if (pitch == 0u)
        pitch = (u32)w * DE_BPP;

#if MORAL_EMU_DEDICATED_THREAD
    de_guest_fb_pull_requested = 1;
    if (MTK != NULL)
        uc_emu_stop(MTK);
    return;
#else
    if (de_read_fb(srcBuf, pitch, w, h) != 0)
    {
        de_deferred_pending = 0;
        return;
    }

    if (w >= DE_PANEL_W - 20u && h >= DE_PANEL_H - 80u)
    {
        u32 rowBytes = (u32)w * DE_BPP;
        u32 step = (rowBytes < 16u) ? 2u : 16u;
        u32 cnt = 0, white = 0;
        for (u32 off = 0; off < (u32)h * rowBytes && cnt < 64u; off += step)
        {
            cnt++;
            if (off + 1u < (u32)h * rowBytes &&
                Lcd_Cache_Buffer[off] == 0xFFu && Lcd_Cache_Buffer[off + 1u] == 0xFFu)
                white++;
        }
        if (cnt > 0 && white * 10u >= cnt * 9u)
        {
            de_deferred_pending = 0;
            return;
        }
    }

    if (w == DE_PANEL_W && h == DE_PANEL_H)
        de_blit_to_sdl(0, 0, w, h, (u32)w * DE_BPP);
    else
    {
        u16 dstX, dstY;
        de_resolve_blit_dest(srcBuf, &dstX, &dstY);
        de_blit_to_sdl(dstX, dstY, w, h, (u32)w * DE_BPP);
    }

    de_deferred_pending = 0;
    de_deferred_last_draw = now;
    Lcd_Need_Update = 1;
    Perf_LcdRefreshCount++;
#endif
}

void de_emulator_periodic_refresh(void)
{
    if (!De_PeriodicRefreshAllowed)
        return;

#if MORAL_EMU_DEDICATED_THREAD
    de_periodic_fb_pull_requested = 1;
    if (MTK != NULL)
        uc_emu_stop(MTK);
#else
    {
        uint64_t now = moral_get_ticks_ms();
        uint64_t idle_threshold = 100;
        if (De_LastTriggerTime != 0 && (now - De_LastTriggerTime) < idle_threshold)
            return;

        /* 周期拉帧：优先 DE_Layer0_Ptr（OP 主层），回退 Lcd_FullScreen_Ptr */
        u32 srcBuf = (DE_Layer0_Ptr >= 0x1000u && DE_Layer0_Ptr < 0x8000000u)
                     ? DE_Layer0_Ptr : Lcd_FullScreen_Ptr;
        if (srcBuf < 0x1000u || srcBuf >= 0x8000000u)
            srcBuf = Lcd_Buffer_Ptr;
        if (srcBuf < 0x1000u || srcBuf >= 0x8000000u)
            return;

        {
            u8 probe[8];
            if (uc_mem_read(MTK, srcBuf, probe, 8) == UC_ERR_OK)
            {
                u32 w1 = *(u32 *)(probe + 4);
                if (w1 >= 0xD0000u && w1 < 0x2000000u)
                    return;
            }
        }

        {
            int got = 0;
#if MORAL_DE_LAYER_COMPOSITE
            if (de_multilayer_composite_needed())
                got = (de_read_fb_composite_into(Lcd_Periodic_Buffer, sizeof(Lcd_Periodic_Buffer)) == 0);
#endif
            if (!got)
            {
                u32 pitch = de_panel_read_pitch_from_base(srcBuf);

                if (de_read_fb_to(Lcd_Periodic_Buffer, sizeof(Lcd_Periodic_Buffer),
                                  srcBuf, pitch, DE_PANEL_W, DE_PANEL_H) != 0)
                    return;
            }
        }

        de_blit_from(Lcd_Periodic_Buffer, 0, 0, DE_PANEL_W, DE_PANEL_H, DE_PANEL_W * DE_BPP);
        Lcd_Need_Update = 1;
        Perf_LcdRefreshCount++;
        de_periodic_call_cnt++;
        if (MORAL_LOG_HOT_PATH && (de_periodic_call_cnt <= 5 || (de_periodic_call_cnt % 200) == 0))
            printf("[LCD-REFRESH] periodic #%u buf=0x%x\n", de_periodic_call_cnt, srcBuf);
    }
#endif
}

void de_lcd_apply_pending_host_blit(void)
{
    if (!s_lcd_host_blit_pending)
        return;
    s_lcd_host_blit_pending = 0;
    de_blit_to_sdl(s_lcd_host_blit_dstX, s_lcd_host_blit_dstY, s_lcd_host_blit_w, s_lcd_host_blit_h,
                   s_lcd_host_blit_pitch);
    Lcd_Need_Update = 1;
}

#if MORAL_EMU_DEDICATED_THREAD
void de_emulator_service_guest_fb_pull(void)
{
    u32 srcBuf;
    u32 pitch;
    u16 w, h;
    uint64_t now;
    u16 dstX, dstY;

    if (de_guest_fb_pull_requested)
    {
        de_guest_fb_pull_requested = 0;
        if (de_deferred_pending)
        {
            now = moral_get_ticks_ms();
            srcBuf = de_deferred_src_buf;
            pitch = de_deferred_pitch;
            w = de_deferred_w;
            h = de_deferred_h;

            if (w < (DE_PANEL_W - 20u) || h < (DE_PANEL_H - 80u))
            {
                u32 expandBase = de_guest_fb_base_containing(de_deferred_src_buf);
                if (expandBase != 0u)
                {
                    srcBuf = expandBase;
                    pitch = de_guest_pitch_for_full_panel_read(expandBase);
                    w = (u16)DE_PANEL_W;
                    h = (u16)DE_PANEL_H;
                }
            }

            if (srcBuf < 0x1000u || srcBuf >= 0x8000000u || w == 0 || h == 0)
            {
                de_deferred_pending = 0;
            }
            else
            {
                if (pitch == 0u)
                    pitch = (u32)w * DE_BPP;

                if (de_read_fb(srcBuf, pitch, w, h) != 0)
                {
                    de_deferred_pending = 0;
                }
                else
                {
                    if (w >= DE_PANEL_W - 20u && h >= DE_PANEL_H - 80u)
                    {
                        u32 rowBytes = (u32)w * DE_BPP;
                        u32 step = (rowBytes < 16u) ? 2u : 16u;
                        u32 cnt = 0, white = 0;
                        for (u32 off = 0; off < (u32)h * rowBytes && cnt < 64u; off += step)
                        {
                            cnt++;
                            if (off + 1u < (u32)h * rowBytes &&
                                Lcd_Cache_Buffer[off] == 0xFFu &&
                                Lcd_Cache_Buffer[off + 1u] == 0xFFu)
                                white++;
                        }
                        if (cnt > 0 && white * 10u >= cnt * 9u)
                        {
                            de_deferred_pending = 0;
                        }
                        else
                        {
                            goto de_stage_deferred_blit;
                        }
                    }
                    else
                    {
                    de_stage_deferred_blit:
                        if (w == DE_PANEL_W && h == DE_PANEL_H)
                        {
                            s_lcd_host_blit_dstX = 0;
                            s_lcd_host_blit_dstY = 0;
                        }
                        else
                        {
                            de_resolve_blit_dest(srcBuf, &dstX, &dstY);
                            s_lcd_host_blit_dstX = dstX;
                            s_lcd_host_blit_dstY = dstY;
                        }
                        s_lcd_host_blit_w = w;
                        s_lcd_host_blit_h = h;
                        s_lcd_host_blit_pitch = (u32)w * DE_BPP;
                        s_lcd_host_blit_pending = 1;
                        de_deferred_last_draw = now;
                        Perf_LcdRefreshCount++;
                        de_deferred_pending = 0;
                    }
                }
            }
        }
    }

    if (de_periodic_fb_pull_requested)
    {
        de_periodic_fb_pull_requested = 0;
        if (!De_PeriodicRefreshAllowed)
            return;

        now = moral_get_ticks_ms();
        if (De_LastTriggerTime != 0 && (now - De_LastTriggerTime) < 100)
            return;

        srcBuf = (DE_Layer0_Ptr >= 0x1000u && DE_Layer0_Ptr < 0x8000000u)
                 ? DE_Layer0_Ptr : Lcd_FullScreen_Ptr;
        if (srcBuf < 0x1000u || srcBuf >= 0x8000000u)
            srcBuf = Lcd_Buffer_Ptr;
        if (srcBuf < 0x1000u || srcBuf >= 0x8000000u)
            return;

        {
            u8 probe[8];
            if (uc_mem_read(MTK, srcBuf, probe, 8) == UC_ERR_OK)
            {
                u32 w1 = *(u32 *)(probe + 4);
                if (w1 >= 0xD0000u && w1 < 0x2000000u)
                    return;
            }
        }

        pitch = de_panel_read_pitch_from_base(srcBuf);
        {
            int got = 0;
#if MORAL_DE_LAYER_COMPOSITE
            if (de_multilayer_composite_needed())
                got = (de_read_fb_composite_full() == 0);
#endif
            if (!got && de_read_fb(srcBuf, pitch, DE_PANEL_W, DE_PANEL_H) != 0)
                return;
        }

        s_lcd_host_blit_dstX = 0;
        s_lcd_host_blit_dstY = 0;
        s_lcd_host_blit_w = (u16)DE_PANEL_W;
        s_lcd_host_blit_h = (u16)DE_PANEL_H;
        s_lcd_host_blit_pitch = pitch;
        s_lcd_host_blit_pending = 1;
        Perf_LcdRefreshCount++;
        de_periodic_call_cnt++;
        if (MORAL_LOG_HOT_PATH && (de_periodic_call_cnt <= 5 || (de_periodic_call_cnt % 200) == 0))
            printf("[LCD-REFRESH] periodic #%u buf=0x%x\n", de_periodic_call_cnt, srcBuf);
    }
}
#endif /* MORAL_EMU_DEDICATED_THREAD */

void hookRamCallBack(uc_engine *uc, uc_mem_type type, uint64_t address, uint32_t size, int64_t value, u32 data)
{
    (void)uc;
    (void)data;

    u32 tmp;
    switch (address)
    {
    case 0x74006CA8:
        if (type == UC_MEM_WRITE)
        {
            if (value == 0x3333)
            {
                SPI_CMD_Buf_Idx = 0;
            }
            else if (value == 0x5555)
            {
                SPI_CMD_Buf_Idx = 0;
            }
        }
        break;
    case 0x74006C10:
        if (type == UC_MEM_WRITE)
        {
            SPI_CMD_Buf[SPI_CMD_Buf_Idx] = (u8)value;
            SPI_CMD_Buf_Idx++;
        }
        break;
    case 0x74006C14:
        if (type == UC_MEM_READ)
        {
            if (SPI_CMD_Buf_Idx > 0)
            {
                if (SPI_CMD_Buf_Idx == 3)
                {
                    if (SPI_CMD_Buf[0] == 0x0 && SPI_CMD_Buf[1] == 0x4 && SPI_CMD_Buf[2] == 0xff)
                    {
                        SPI_DATA_Buf_Idx = 1;
                        SPI_DATA_Buf[0] = 1;
                    }
                    else if (SPI_CMD_Buf[0] == 0x0 && SPI_CMD_Buf[1] == 0x8 && SPI_CMD_Buf[2] == 0xff)
                    {
                        SPI_DATA_Buf_Idx = 2;
                        SPI_DATA_Buf[0] = 0xff;
                        SPI_DATA_Buf[1] = 0xff;
                    }
                }
            }
            uc_mem_write(MTK, (u32)address, &SPI_DATA_Buf[--SPI_DATA_Buf_Idx], 4);
        }
        break;
    case 0x74006C30:
        break;
    case 0x74006C54:
        if (type == UC_MEM_READ)
        {
            tmp = 1;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x74006C58:
        if (type == UC_MEM_READ)
        {
            tmp = 1;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;

    case 0x34000094:
        if (type == UC_MEM_READ)
        {
            tmp = 0;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x74006480:
        if (type == UC_MEM_READ)
        {
            tmp = 2;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x34002C44:
        if (type == UC_MEM_READ)
        {
            DrvTimerStdaTimerGetTick++;
            uc_mem_write(MTK, (u32)address, &DrvTimerStdaTimerGetTick, 4);
        }

        break;
    case 0x34002C04:
        if (type == UC_MEM_WRITE)
        {
            halTimerOutLength = value & 0xffffff;
            if (halTimerOutLength == 0)
                halTimerCnt = 0;
            else if (halTimerCnt >= halTimerOutLength)
                halTimerCnt %= halTimerOutLength;
        }
        break;
    case 0x34002C08:
        if (type == UC_MEM_READ)
        {
            uc_mem_write(MTK, address, &halTimerCnt, 4);
        }
        break;
    case 0x34002C2C:
        if (type == UC_MEM_WRITE)
        {
            if ((value & 0x10) == 0x10)
                halTimerIntStatus = 1;
        }
        break;
    case 0x34002C34:
        if (type == UC_MEM_WRITE)
        {
            if ((value & 0x10) == 0x10)
                halTimerIntStatus = 0;
        }
        break;
    case 0x34002c00:
        if (type == UC_MEM_READ)
        {
            halTimerCnt = halTimerCnt == 0 ? 2047 : 0;
            uc_mem_write(MTK, (u32)address, &halTimerCnt, 4);
        }
        break;
    case 0x74003094:
        if (type == UC_MEM_WRITE)
        {
            static u32 fmt_cnt = 0;
            fmt_cnt++;
            if (MORAL_LOG_HOT_PATH && fmt_cnt <= 10)
                printf("[lcd]buff_format[%x] #%u\n", (u32)value, fmt_cnt);
        }
        break;
    case 0x7400309C:
        if (type == UC_MEM_WRITE)
        {
            static u32 layer_sel_cnt = 0;
            layer_sel_cnt++;
            DE_LayerSel = (u32)value;
            if (MORAL_LOG_HOT_PATH && (layer_sel_cnt <= 10 || (layer_sel_cnt % 500) == 0))
                printf("[lcd]layer_sel[%x] #%u\n", (u32)value, layer_sel_cnt);
        }
        break;
    case 0x7400303C:
        if (type == UC_MEM_WRITE)
            DE_DispLayerWidthMask = (u32)value;
        break;
    /* === DE Layer 缓冲指针寄存器 ===
     * IDA HalDispSetBufInfo 写序：先 W/H/pitch，再写 hi(ptr+4)，最后写 lo(ptr+0)。
     * main.c 钩住了 HalDispTransAddr(0x1e574)，直接把原始物理地址写入，不做减法。
     * 因此寄存器里存的已经是完整 GPA（>= 0xC000000），不需要额外加偏移。
     * 寄存器宽度 16位，hi/lo 各16位拼成完整32位地址。
     */
    /* Layer0 hi (先写) */
    case 0x74003044:
        if (type == UC_MEM_WRITE)
        {
            DE_Layer0_Ptr &= 0x0000ffffu;
            DE_Layer0_Ptr |= (((u32)value & 0xffffu) << 16);
        }
        break;
    /* Layer0 lo (后写，地址完整)
     * 注意：此值是帧缓冲 + 脏区偏移后的地址（IDA：HalDispSetLayerBuffer 计算 bufPtr+offset）。
     * Lcd_Buffer_Ptr 跟踪当前 trigger 用的地址。
     * Lcd_FullScreen_Ptr（帧缓冲基址）仅在 FMark/整屏时更新，不在此处设置。
     */
    case 0x74003040:
        if (type == UC_MEM_WRITE)
        {
            DE_Layer0_Ptr &= 0xffff0000u;
            DE_Layer0_Ptr |= ((u32)value & 0xffffu);
            Lcd_Buffer_Ptr = DE_Layer0_Ptr;
            De_PeriodicRefreshAllowed = 1;
        }
        break;
    case 0x74003048:
        if (type == UC_MEM_WRITE)
            DE_Layer0_W = (u32)value;
        break;
    case 0x7400304C:
        if (type == UC_MEM_WRITE)
            DE_Layer0_H = (u32)value;
        break;
    case 0x74003050:
        if (type == UC_MEM_WRITE)
            DE_Layer0_Pitch = (u32)value;
        break;

    /* === DE Layer1 (IDA a1=1, PIP1) === */
    case 0x74003058: /* hi */
        if (type == UC_MEM_WRITE)
        {
            DE_Layer1_Ptr &= 0x0000ffffu;
            DE_Layer1_Ptr |= (((u32)value & 0xffffu) << 16);
        }
        break;
    case 0x74003054: /* lo */
        if (type == UC_MEM_WRITE)
        {
            DE_Layer1_Ptr &= 0xffff0000u;
            DE_Layer1_Ptr |= ((u32)value & 0xffffu);
        }
        break;
    case 0x7400305C:
        if (type == UC_MEM_WRITE)
            DE_Layer1_W = (u32)value;
        break;
    case 0x74003060:
        if (type == UC_MEM_WRITE)
            DE_Layer1_H = (u32)value;
        break;
    case 0x74003064:
        if (type == UC_MEM_WRITE)
            DE_Layer1_Pitch = (u32)value;
        break;

    /* === DE Layer2 (IDA a1=2, PIP2) === */
    case 0x7400306C: /* hi */
        if (type == UC_MEM_WRITE)
        {
            DE_Layer2_Ptr &= 0x0000ffffu;
            DE_Layer2_Ptr |= (((u32)value & 0xffffu) << 16);
        }
        break;
    case 0x74003068: /* lo */
        if (type == UC_MEM_WRITE)
        {
            DE_Layer2_Ptr &= 0xffff0000u;
            DE_Layer2_Ptr |= ((u32)value & 0xffffu);
        }
        break;
    case 0x74003070:
        if (type == UC_MEM_WRITE)
            DE_Layer2_W = (u32)value;
        break;
    case 0x74003074:
        if (type == UC_MEM_WRITE)
            DE_Layer2_H = (u32)value;
        break;
    case 0x74003078:
        if (type == UC_MEM_WRITE)
            DE_Layer2_Pitch = (u32)value;
        break;

    /* === DE Layer3 (IDA a1=3, PIP3) === */
    case 0x74003084: /* hi */
        if (type == UC_MEM_WRITE)
        {
            DE_Layer3_Ptr &= 0x0000ffffu;
            DE_Layer3_Ptr |= (((u32)value & 0xffffu) << 16);
        }
        break;
    case 0x74003080: /* lo */
        if (type == UC_MEM_WRITE)
        {
            DE_Layer3_Ptr &= 0xffff0000u;
            DE_Layer3_Ptr |= ((u32)value & 0xffffu);
        }
        break;
    case 0x74003088:
        if (type == UC_MEM_WRITE)
            DE_Layer3_W = (u32)value;
        break;
    case 0x7400308C:
        if (type == UC_MEM_WRITE)
            DE_Layer3_H = (u32)value;
        break;
    case 0x74003090:
        if (type == UC_MEM_WRITE)
            DE_Layer3_Pitch = (u32)value;
        break;
    case 0x740030D0:
        if (type == UC_MEM_WRITE)
            DE_AlphaHw[0] = (short)((u32)value & 0xFFFFu);
        break;
    case 0x740030D4:
        if (type == UC_MEM_WRITE)
            DE_AlphaHw[1] = (short)((u32)value & 0xFFFFu);
        break;
    case 0x740030D8:
        if (type == UC_MEM_WRITE)
            DE_AlphaHw[2] = (short)((u32)value & 0xFFFFu);
        break;
    case 0x740030DC:
        if (type == UC_MEM_WRITE)
            DE_CKey1[0] = (u16)(u32)value;
        break;
    case 0x740030E0:
        if (type == UC_MEM_WRITE)
            DE_CKey1[1] = (u16)(u32)value;
        break;
    case 0x740030E4:
        if (type == UC_MEM_WRITE)
            DE_CKey1[2] = (u16)(u32)value;
        break;
    case 0x740030E8:
        if (type == UC_MEM_WRITE)
            DE_CKey2[0] = (u16)(u32)value;
        break;
    case 0x740030EC:
        if (type == UC_MEM_WRITE)
            DE_CKey2[1] = (u16)(u32)value;
        break;
    case 0x740030F0:
        if (type == UC_MEM_WRITE)
            DE_CKey2[2] = (u16)(u32)value;
        break;
    case 0x740030F4:
        if (type == UC_MEM_WRITE)
            DE_CKey3[0] = (u16)(u32)value;
        break;
    case 0x740030F8:
        if (type == UC_MEM_WRITE)
            DE_CKey3[1] = (u16)(u32)value;
        break;
    case 0x740030FC:
        if (type == UC_MEM_WRITE)
            DE_CKey3[2] = (u16)(u32)value;
        break;
    case 0x74003100:
        if (type == UC_MEM_WRITE)
            DE_ColorKeyCtrl = (u32)value;
        break;
    case 0x740030A0:
        if (type == UC_MEM_WRITE)
            DE_PipLayer1[0] = (u16)(u32)value;
        break;
    case 0x740030A4:
        if (type == UC_MEM_WRITE)
            DE_PipLayer1[1] = (u16)(u32)value;
        break;
    case 0x740030A8:
        if (type == UC_MEM_WRITE)
            DE_PipLayer1[2] = (u16)(u32)value;
        break;
    case 0x740030AC:
        if (type == UC_MEM_WRITE)
            DE_PipLayer1[3] = (u16)(u32)value;
        break;
    case 0x740030B0:
        if (type == UC_MEM_WRITE)
            DE_PipLayer2[0] = (u16)(u32)value;
        break;
    case 0x740030B4:
        if (type == UC_MEM_WRITE)
            DE_PipLayer2[1] = (u16)(u32)value;
        break;
    case 0x740030B8:
        if (type == UC_MEM_WRITE)
            DE_PipLayer2[2] = (u16)(u32)value;
        break;
    case 0x740030BC:
        if (type == UC_MEM_WRITE)
            DE_PipLayer2[3] = (u16)(u32)value;
        break;
    /* PIP Layer3 位置寄存器（IDA HalDispSetPIP case 3） */
    case 0x740030C0:
        if (type == UC_MEM_WRITE)
            DE_PipLayer3[0] = (u16)(u32)value;
        break;
    case 0x740030C4:
        if (type == UC_MEM_WRITE)
            DE_PipLayer3[1] = (u16)(u32)value;
        break;
    case 0x740030C8:
        if (type == UC_MEM_WRITE)
            DE_PipLayer3[2] = (u16)(u32)value;
        break;
    case 0x740030CC:
        if (type == UC_MEM_WRITE)
            DE_PipLayer3[3] = (u16)(u32)value;
        break;
    case 0x7400313C:
        if (type == UC_MEM_WRITE)
        {
            /*
             * DE trigger（IDA：HalDispForceUpdate 写 0x7400313C=1）。
             * 此时 HalDispSetBufInfo 已把脏区参数写入寄存器：
             *   0x74003040 = bufPtr（帧缓冲 + 脏区左上角偏移）
             *   0x74003048 = 脏区 width
             *   0x7400304C = 脏区 height
             *   0x74003050 = pitch（固定 = 2 × 屏宽 = 480，整屏行距）
             * 所以直接用 DE_Layer0_* 即可，不需要 Lcd_Update_W/H。
             */
            u32 srcBuf = DE_Layer0_Ptr;
            u16 w = (DE_Layer0_W > 0u && DE_Layer0_W <= (u32)DE_PANEL_W) ? (u16)DE_Layer0_W : (u16)DE_PANEL_W;
            u16 h = (DE_Layer0_H > 0u && DE_Layer0_H <= (u32)DE_PANEL_H) ? (u16)DE_Layer0_H : (u16)DE_PANEL_H;
            u32 pitch = (DE_Layer0_Pitch >= (u32)w * DE_BPP && DE_Layer0_Pitch < 8192u)
                        ? DE_Layer0_Pitch : (u32)w * DE_BPP;

            static u32 de_trigger_cnt = 0;
            static u32 boot_desc_buf = 0;
            de_trigger_cnt++;

            /* srcBuf 无效时直接发 IRQ 跳过，不读 Guest 内存 */
            if (srcBuf < 0x1000u || srcBuf >= 0x8000000u || w == 0u || h == 0u)
            {
                tmp = 0xF00;
                uc_mem_write(MTK, 0x74003148u, &tmp, 4);
                lcd_irq_enqueue_throttled();
                break;
            }

            if (de_trigger_cnt == 1 && w >= DE_PANEL_W && h >= DE_PANEL_H)
                boot_desc_buf = srcBuf;

            if (srcBuf == boot_desc_buf && w >= DE_PANEL_W && h >= DE_PANEL_H)
            {
                u8 probe[8];
                if (uc_mem_read(MTK, srcBuf, probe, 8) == UC_ERR_OK)
                {
                    u32 w1 = *(u32 *)(probe + 4);
                    if (w1 >= 0xD0000u && w1 < 0x2000000u)
                    {
                        tmp = 0xF00;
                        uc_mem_write(MTK, 0x74003148u, &tmp, 4);
                        lcd_irq_enqueue_throttled();
                        break;
                    }
                    else
                        boot_desc_buf = 0;
                }
            }

            if (w > DE_PANEL_W)
                w = (u16)DE_PANEL_W;
            if (h > DE_PANEL_H)
                h = (u16)DE_PANEL_H;
            if (pitch == 0u)
                pitch = (u32)w * DE_BPP;

            if (de_trigger_cnt <= 50 || (de_trigger_cnt % 200) == 0)
            {
                printf("[DE-trigger] #%u src=0x%x w=%u h=%u pitch=%u | updXY=(%u,%u) updWH=(%u,%u) | FSP=0x%x FSP2=0x%x\n",
                       de_trigger_cnt, srcBuf, w, h, pitch,
                       Lcd_Update_X, Lcd_Update_Y, Lcd_Update_W, Lcd_Update_H,
                       Lcd_FullScreen_Ptr, Lcd_FullScreen_Ptr2);
            }

            de_try_snapshot_fb_at_de_trigger(srcBuf, pitch, w, h, 0);
            tmp = 0xF00;
            uc_mem_write(MTK, 0x74003148u, &tmp, 4);
            lcd_irq_enqueue_throttled();
        }
        break;
    case 0x74003140:
        if (type == UC_MEM_WRITE)
        {
            u32 pc_val;
            uc_reg_read(MTK, UC_ARM_REG_PC, &pc_val);
            u32 is_reset = (pc_val >= 0x1ec76u && pc_val <= 0x1ecceu);

            if (value == 0)
            {
                static u32 fmark_clear_cnt = 0;
                fmark_clear_cnt++;
                if (MORAL_LOG_HOT_PATH && fmark_clear_cnt <= 10)
                    printf("[FMark-clear] #%u pc=0x%x %s\n", fmark_clear_cnt, pc_val,
                           is_reset ? "(HalDispReset)" : "(SWFMark)");
                if (is_reset)
                {
                    /* HalDispReset：清所有 PIP 子层镜像，避免旧寄存器在下一帧被误合成。
                     * Layer0 (OP 主层) 由下一次 HalDispSetBufInfo(0,...) 覆盖，此处不清除。 */
                    DE_Layer1_Ptr = 0; DE_Layer1_W = 0; DE_Layer1_H = 0; DE_Layer1_Pitch = 0;
                    DE_Layer2_Ptr = 0; DE_Layer2_W = 0; DE_Layer2_H = 0; DE_Layer2_Pitch = 0;
                    DE_Layer3_Ptr = 0; DE_Layer3_W = 0; DE_Layer3_H = 0; DE_Layer3_Pitch = 0;
                    DE_PipLayer1[0] = DE_PipLayer1[1] = DE_PipLayer1[2] = DE_PipLayer1[3] = 0;
                    DE_PipLayer2[0] = DE_PipLayer2[1] = DE_PipLayer2[2] = DE_PipLayer2[3] = 0;
                    DE_PipLayer3[0] = DE_PipLayer3[1] = DE_PipLayer3[2] = DE_PipLayer3[3] = 0;
                    DE_CKey1[0] = DE_CKey1[1] = DE_CKey1[2] = 0;
                    DE_CKey2[0] = DE_CKey2[1] = DE_CKey2[2] = 0;
                    DE_CKey3[0] = DE_CKey3[1] = DE_CKey3[2] = 0;
                    DE_AlphaHw[0] = DE_AlphaHw[1] = DE_AlphaHw[2] = 0;
                    DE_ColorKeyCtrl = 0;
                    DE_DispLayerWidthMask = 0;
                    Lcd_Update_W = 0;
                    Lcd_Update_H = 0;
                }
                tmp = 0xF00;
                uc_mem_write(MTK, 0x74003148u, &tmp, 4);
                break;
            }

            static u32 fmark_trigger_cnt = 0;
            fmark_trigger_cnt++;
            if (MORAL_LOG_HOT_PATH && (fmark_trigger_cnt <= 20 || (fmark_trigger_cnt % 100) == 0))
            {
                printf("[FMark-trigger] #%u pc=0x%x %s\n", fmark_trigger_cnt, pc_val,
                       is_reset ? "(HalDispReset)" : "(SWFMark)");
            }

            if (!is_reset)
            {
                /* FMark 来自 HalDispSWFMarkTrigger，此时 Layer0 寄存器已写了脏区参数。
                 * Lcd_Buffer_Ptr (= DE_Layer0_Ptr) 是脏区偏移后的地址，不是帧缓冲基址。
                 * 不更新 Lcd_FullScreen_Ptr——它已在 de_try_snapshot 整屏路径里正确设置。 */
                De_PeriodicRefreshAllowed = 1;
                lcd_irq_enqueue_throttled();
            }
            tmp = 0xF00;
            uc_mem_write(MTK, 0x74003148u, &tmp, 4);
        }
        break;
    case 0x740031A0:
        if (type == UC_MEM_READ)
        {
            tmp = 3;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;

    case 0x34000424:
        if (type == UC_MEM_READ)
        {
            tmp = 3;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;

    case 0x3400500C:
    case 0x3400540C:
        if (type == UC_MEM_READ)
        {
            tmp = 0x20;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x740050C0:
        if (type == UC_MEM_WRITE)
        {
            if ((value >> 12) & 1)
            {
                tmp = 4;
                uc_mem_write(MTK, 0x74005100, &tmp, 4);
            }
        }
        break;
    case 0x34002C10:
        if (type == UC_MEM_WRITE)
        {
            if (value == 0x11FFF || value == 0x10FFF)
            {
                if (nand_op_pending)
                {
                    FICE_Status = 0x02000200;
                    nand_op_pending = 0;
                }
            }
        }
        break;
    case NC_MIE_EVENT:
        if (type == UC_MEM_READ)
        {
            uc_mem_write(MTK, (u32)address, &FICE_Status, 4);
        }
        else
        {
            if ((u32)value == 0u)
                FICE_Status = 0;
            else if (((u32)value & FICE_Status) == (u32)value)
                FICE_Status &= ~(u32)value;
            else
                FICE_Status = value;
        }
        break;
    case 0x74005004:
        if (type == UC_MEM_READ)
        {
            tmp = 0;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x74005008:
        if (type == UC_MEM_READ)
        {
            tmp = 0x20;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x74005028:
        if (type == UC_MEM_READ)
        {
            tmp = 0x80;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    /* NAND FCIE：命令字与参数经 AUX 寄存器写入，NC_CTRL 置位后在本文件内填 DMA/CIFD */
    case NC_AUXREG_ADR:
        if (type == UC_MEM_WRITE)
        {
            nandFlashCMD = value;
        }
        break;
    case NC_AUXREG_DAT:
        if (type == UC_MEM_WRITE)
        {
            nandFlashCMDData[nandFlashCmdIdx++] = (u16)value;
            if (nandFlashCmdIdx >= 32)
            {
                printf("[nand] NC_AUXREG_DAT overflow idx=%d val=0x%x pc=0x%x\n",
                       nandFlashCmdIdx, (u16)value, lastAddress);
                nandFlashCmdIdx = 31;
            }
        }
        break;
    case 0x7400515c:
        if (type == UC_MEM_WRITE)
        {
            if (value == 3)
            {
                nandFlashCmdIdx = 0;
                my_memset(nandFlashCMDData, 0, sizeof(nandFlashCMDData));
            }
        }
        break;
    case 0x74003124:
        if (type == UC_MEM_WRITE)
        {
            LCD_CMD_Data = value;
        }
        break;
    case 0x74003128:
        if (type == UC_MEM_WRITE)
        {
            LCD_CMD_Data &= 0xffff;
            LCD_CMD_Data |= (value << 16);
        }
        break;
    case 0x74003130:
        if (type == UC_MEM_READ)
        {
            if (LCD_CMD_Data == 0)
                value = 0x65;
            else if (LCD_CMD_Data == 0xef)
                value = 0x2;
            uc_mem_write(MTK, (u32)address, &value, 4);
        }
        break;
    case 0x7400511C:
        if (type == UC_MEM_WRITE)
        {
            nandParamSect = value;
        }

        break;
    case NC_CTRL:
        if (type == UC_MEM_WRITE)
        {
            if ((value & 1) == 1)
            {
                nand_op_pending = 1;
                u16 SectorInPage = nandFlashCMDData[0];
                u32 PhyRowIdx = (nandFlashCMDData[1] | (nandFlashCMDData[2] << 16));
                u32 destRowIdx = (nandFlashCMDData[4] | (nandFlashCMDData[5] << 16));
                u32 act = nandFlashCMDData[4] | (nandFlashCMDData[5] << 16);

                if (nandFlashCMD == 0x20)
                {

                    if (act == 0x88889880)
                    {
                        nandPage512 *pool = (nandPage512 *)NandFlashCard;
                        nandPage512 *p = pool + (PhyRowIdx * 4);
                        uc_mem_write(MTK, nandDmaBuffPtr, p->pageBuff, 512);
                        for (tmp = 0; tmp < 8; tmp++)
                        {
                            nandSpareBuff[tmp] = p->spareBuff[tmp];
                        }
                        uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandSpareBuff, 32);
                    }
                    else if (act == 0x88988001)
                    {
                        nandPage2048 *p = ((nandPage2048 *)NandFlashCard) + PhyRowIdx;

                        if (nandParamSect == 1)
                            nand_nc_read_page_sectors_contig(p, nandDmaBuffPtr, 0, 1);
                        else if (nandParamSect == 0x81)
                            nand_nc_read_page_sectors_contig(p, nandDmaBuffPtr, 1, 1);
                        else if (nandParamSect == 0x101)
                            nand_nc_read_page_sectors_contig(p, nandDmaBuffPtr, 2, 1);
                        else if (nandParamSect == 0x181)
                            nand_nc_read_page_sectors_contig(p, nandDmaBuffPtr, 3, 1);
                        else if (nandParamSect == 0x85)
                            nand_nc_read_page_sectors_contig(p, nandDmaBuffPtr, 1, 3);
                        else if (nandParamSect == 0x5)
                            nand_nc_read_page_sectors_contig(p, nandDmaBuffPtr, 0, 3);
                        else if (nandParamSect == 0x3)
                            nand_nc_read_page_sectors_contig(p, nandDmaBuffPtr, 0, 2);
                        else if (nandParamSect == 0x103u)
                        {
                            u32 fs = 1u, ns = 3u;
                            if (SectorInPage > 0u && SectorInPage <= 0x600u && (SectorInPage % 512u) == 0u)
                                fs = SectorInPage / 512u;
                            if (fs + ns > 4u)
                                ns = 4u - fs;
                            nand_nc_read_page_sectors_contig(p, nandDmaBuffPtr, fs, ns);
                        }
                        else
                        {
                            u32 firstSector = 0u;
                            u32 sectorCount = 0u;
                            if (nand_nc_resolve_sector_window(nandParamSect, SectorInPage, &firstSector, &sectorCount))
                            {
                                nand_nc_read_page_sectors_contig(p, nandDmaBuffPtr, firstSector, sectorCount);
                            }
                            else
                            {
                                static u32 nand_sec_fb_log;
                                if (nand_sec_fb_log < 24u)
                                {
                                    nand_sec_fb_log++;
                                    printf("[nand] unresolved cmd20 read act88988001 param=0x%x SectorInPage=0x%x phy=%u\n",
                                           nandParamSect, SectorInPage, PhyRowIdx);
                                }
                            }
                        }
                    }
                    else if (act == 0xd800690)
                    {
                        nandPage2048 *p = ((nandPage2048 *)NandFlashCard) + PhyRowIdx;
                        {
                            u32 firstSector = 0u;
                            u32 sectorCount = 0u;
                            if (nand_nc_resolve_sector_window(nandParamSect, SectorInPage, &firstSector, &sectorCount))
                            {
                                nand_nc_write_page_sectors_contig(p, nandDmaBuffPtr, firstSector, sectorCount);
                            }
                            else
                            {
                                static u32 nand_sec_wr_warn;
                                if (nand_sec_wr_warn < 24u)
                                {
                                    nand_sec_wr_warn++;
                                    printf("[nand] unresolved cmd20 write act=0x%x param=0x%x SectorInPage=0x%x phy=%u\n",
                                           act, nandParamSect, SectorInPage, PhyRowIdx);
                                }
                            }
                        }
                    }
                    else if (act == 0 || act == 0x8880 || act == 0x810d800a)
                    {
                    }
                    else
                    {
                        static u32 nand_act20_warn;
                        if (nand_act20_warn < 16u)
                        {
                            nand_act20_warn++;
                            printf("[nand] unhandled cmd 0x20 act=0x%x pc=0x%x phy=%u (no data filled)\n",
                                   act, lastAddress, PhyRowIdx);
                        }
                    }
                }
                else if (nandFlashCMD == 0x19)
                {
                    if (act == 0)
                    {
                        uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandIdInfo, 32);
                    }
                    else
                    {
                        static u32 nand_act19_warn;
                        if (nand_act19_warn < 16u)
                        {
                            nand_act19_warn++;
                            printf("[nand] unhandled cmd 0x19 act=0x%x pc=0x%x\n", act, lastAddress);
                        }
                    }
                }
                else if (nandFlashCMD == 0x18)
                {
                    int cmd7 = nandFlashCMDData[7];

                    if (act == 0xb0988001)
                    {
                        u16 pageCnt = nandFlashCMDData[7] + 1;
                        for (u32 cnt = 0; cnt < pageCnt; cnt++)
                        {
                            u32 phyIdx = PhyRowIdx + cnt;
                            nandPage2048 *p = ((nandPage2048 *)NandFlashCard) + phyIdx;
                            u32 virtAddr = nandDmaBuffPtr + (cnt * 2048);
                            uc_mem_write(MTK, virtAddr, p->pageBuff, 2048);
                            for (tmp = 0; tmp < 32; tmp++)
                            {
                                nandSpareBuff[tmp] = p->spareBuff[tmp];
                            }
                            uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE + (cnt * 128), nandSpareBuff, 128);
                        }
                    }
                    else if (act == 0xd800690)
                    {
                        u16 pageCnt = nandFlashCMDData[8] + 1;
                        for (u32 cnt = 0; cnt < pageCnt; cnt++)
                        {
                            u32 phyIdx = PhyRowIdx + cnt;
                            u32 virtAddr = nandDmaBuffPtr + (cnt * 2048);
                            nandPage2048 *p = ((nandPage2048 *)NandFlashCard) + phyIdx;
                            uc_mem_read(MTK, virtAddr, p->pageBuff, 2048);
                        }
                    }
                    else if (cmd7 == 0xa082)
                    {
                        u32 cntAll = 0;
                        uc_mem_read(MTK, 0x74005110, &cntAll, 4);
                        cntAll += 1;

                        for (u32 cnt = 0; cnt < cntAll; cnt++)
                        {
                            nandPage2048 *src = ((nandPage2048 *)NandFlashCard) + PhyRowIdx + cnt;
                            nandPage2048 *dest = ((nandPage2048 *)NandFlashCard) + destRowIdx + cnt;
                            my_memcpy(dest, src, sizeof(nandPage2048));
                        }
                    }
                    else
                    {
                        static u32 nand_act18_warn;
                        if (nand_act18_warn < 16u)
                        {
                            nand_act18_warn++;
                            printf("[nand] unhandled cmd 0x18 act=0x%x pc=0x%x\n", act, lastAddress);
                        }
                    }
                }
                else
                {
                    static u32 nand_cmd_warn;
                    if (nand_cmd_warn < 16u)
                    {
                        nand_cmd_warn++;
                        printf("[nand] unhandled cmd 0x%x act=0x%x pc=0x%x\n",
                               nandFlashCMD, act, lastAddress);
                    }
                }
                /*
                 * NAND 操作在此已同步完成（数据搬运到 Guest 内存）。
                 * 立即置 FICE_Status，使 NC_WaitComplete 的轮询立刻通过。
                 * 不等 0x34002C10 异步触发——那条路径可能在 WaitComplete 之后才到。
                 */
                FICE_Status |= 0x02000200u;
                nand_op_pending = 0;
            }
        }
        break;
    case 0x74006EC0:
        if (type == UC_MEM_WRITE)
        {
            HalVMMPControl = value;
            if ((HalVMMPControl & 2) == 2)
            {
                HalVMMPControl |= 8;
            }
        }
        else
        {
            uc_mem_write(MTK, (u32)address, &HalVMMPControl, 4);
        }
        break;
    case 0x74005044:
        if (type == UC_MEM_WRITE && value != 0)
        {
            uc_mem_read(MTK, 0x74005200, &SD_CMD_Buff, 32);
            u32 *p = SD_CMD_Buff;
            my_memset(SD_CMD_RSP_Buff, 0, sizeof(SD_CMD_RSP_Buff));
            /* 连写第二次仍为 0xDEDE RSP，勿当新 CMD 解析（会冲掉 RSP、破坏 FAT） */
            if (((p[0] >> 16) & 0xffffu) == 0xdedeu)
            {
                goto sd_cmd_done;
            }
            if (p[0] == 0x40u && p[1] == 0u && p[2] == 0u)
            {
                SD_CMD_RSP_Buff[0] = 0xdede0000;
                SD_CMD_RSP_Buff[1] = 0xdede0100;
                SD_CMD_RSP_Buff[2] = 0xdede005a;
            }
            else if ((p[0] == 0x48) && p[1] == 0x100 && p[2] == 0x5a)
            {
                SD_CMD_RSP_Buff[0] = 0xdede0000;
                SD_CMD_RSP_Buff[1] = 0xdede0100;
                SD_CMD_RSP_Buff[2] = 0xdede005a;
            }
            else if ((p[0] == 0x4069) && p[1] == 0x8001 && p[2] == 0)
            {
                SD_CMD_RSP_Buff[0] = 0xdedec000;
                SD_CMD_RSP_Buff[1] = 0xdede0000;
            }
            else if ((p[0] == 0x77 && p[1] == 0) || (p[0] == 0x73 && p[1] == 0) || (p[0] == 0x377 && p[1] == 8) || (p[0] == 0x347 && p[1] == 8) || (p[0] == 0x46 && p[1] == 0 && p[2] == 2))
            {
                SD_CMD_RSP_Buff[0] = 0xdede0000;
                SD_CMD_RSP_Buff[1] = 0xdede0700;
            }
            else if (p[0] == 0x43 && p[1] == 0)
            {
                SD_CMD_RSP_Buff[0] = 0x01020304;
                SD_CMD_RSP_Buff[1] = 0x05060708;
            }
            else if (p[0] == 0xff49 && p[1] == 8)
            {
                u32 csd_regs[9];
                sd_fill_csd_v2_regs(csd_regs);
                uc_mem_write(MTK, 0x74005200, csd_regs, sizeof(csd_regs));
                goto sd_cmd_done;
            }
            else if (p[0] == 0x42 && p[1] == 0)
            {
                SD_CMD_RSP_Buff[0] = 0x01020304;
                SD_CMD_RSP_Buff[1] = 0x05060708;
            }
            else if ((p[0] & 0x3f) == 0x06)
            {
                u16 miu_lo16_c6 = 0, miu_hi16_c6 = 0;
                uc_mem_read(MTK, 0x74005070, &miu_lo16_c6, 2);
                uc_mem_read(MTK, 0x74005074, &miu_hi16_c6, 2);
                u32 dma_addr = (((u32)miu_hi16_c6 << 16) | miu_lo16_c6) + 0xC000000;
                u8 sw_status[64];
                my_memset(sw_status, 0, sizeof(sw_status));
                sw_status[13] = 0x03;
                sw_status[16] = 0x01;
                uc_mem_write(MTK, dma_addr, sw_status, 64);
                SD_CMD_RSP_Buff[0] = 0xdede0000;
                SD_CMD_RSP_Buff[1] = 0xdede0900;
            }
            else if ((p[0] & 0x3f) == 0x09)
            {
                u32 csd_regs[9];
                sd_fill_csd_v2_regs(csd_regs);
                uc_mem_write(MTK, 0x74005200, csd_regs, sizeof(csd_regs));
                goto sd_cmd_done;
            }
            else if ((p[0] & 0x3f) == 0x11 || (p[0] & 0x3f) == 0x12)
            {
                u16 miu_lo16 = 0, miu_hi16 = 0;
                u32 blk_cnt = 0;
                uc_mem_read(MTK, 0x74005070, &miu_lo16, 2);
                uc_mem_read(MTK, 0x74005074, &miu_hi16, 2);
                uc_mem_read(MTK, 0x7400502c, &blk_cnt, 4);
                u32 miu_addr = ((u32)miu_hi16 << 16) | miu_lo16;
                u32 dma_addr;
                if (g_sd_dma_phys_addr != 0)
                    dma_addr = g_sd_dma_phys_addr;
                else if (miu_addr != 0)
                    dma_addr = miu_addr + 0xC000000;
                else
                    dma_addr = 0xC000000;
                u32 sector_addr = p[1];
                blk_cnt &= 0xfff;
                if (blk_cnt == 0)
                    blk_cnt = 1;
                u32 byte_count = blk_cnt * 512;
                unsigned long long file_offset = (unsigned long long)sector_addr * 512ULL;
                  if (MORAL_LOG_SD_IO)
                      printf("[SD-CMD17/18] sector=%u blk=%u miu=0x%08x dma=0x%08x\n",
                          sector_addr, blk_cnt, miu_addr, dma_addr);
                u8 *buf = readSDFile(file_offset, byte_count);
                if (buf != NULL)
                {
                    uc_mem_write(MTK, dma_addr, buf, byte_count);
                    SDL_free(buf);
                }
                SD_CMD_RSP_Buff[0] = 0xdede0000;
                SD_CMD_RSP_Buff[1] = 0xdede0900;
            }
            else if ((p[0] & 0x3f) == 0x18 || (p[0] & 0x3f) == 0x19)
            {
                u16 miu_lo16_wr = 0, miu_hi16_wr = 0;
                u32 blk_cnt = 0;
                uc_mem_read(MTK, 0x74005070, &miu_lo16_wr, 2);
                uc_mem_read(MTK, 0x74005074, &miu_hi16_wr, 2);
                uc_mem_read(MTK, 0x7400502c, &blk_cnt, 4);
                u32 miu_addr_wr_val = ((u32)miu_hi16_wr << 16) | miu_lo16_wr;
                u32 dma_addr = (g_sd_dma_phys_addr != 0)
                                   ? g_sd_dma_phys_addr
                                   : (miu_addr_wr_val != 0 ? miu_addr_wr_val + 0xC000000 : 0xC000000);
                u32 sector_addr = p[1];
                blk_cnt &= 0xfff;
                if (blk_cnt == 0)
                    blk_cnt = 1;
                u32 byte_count = blk_cnt * 512;
                unsigned long long file_offset = (unsigned long long)sector_addr * 512ULL;
                u8 *dma_buf = (u8 *)SDL_malloc(byte_count);
                if (dma_buf != NULL)
                {
                    uc_mem_read(MTK, dma_addr, dma_buf, byte_count);
                    writeSDFile(dma_buf, file_offset, byte_count);
                    SDL_free(dma_buf);
                }
                SD_CMD_RSP_Buff[0] = 0xdede0000;
                SD_CMD_RSP_Buff[1] = 0xdede0900;
                  if (MORAL_LOG_SD_IO)
                      printf("[SD-CMD24/25] sector=%u blk=%u dma=0x%08x\n",
                          sector_addr, blk_cnt, dma_addr);
            }
            else if ((p[0] & 0x3f) == 0x0c)
            {
                SD_CMD_RSP_Buff[0] = 0xdede0000;
                SD_CMD_RSP_Buff[1] = 0xdede0900;
            }
            else if ((p[0] & 0x3f) == 0x0d)
            {
                SD_CMD_RSP_Buff[0] = 0xdede0000;
                SD_CMD_RSP_Buff[1] = 0xdede0900;
            }
            else
            {
                static u32 sd_fcie_unknown;
                if (sd_fcie_unknown < 32u)
                {
                    sd_fcie_unknown++;
                    printf("[SD-FCIE] unhandled p0=0x%x p1=0x%x p2=0x%x\n", p[0], p[1], p[2]);
                }
                SD_CMD_RSP_Buff[0] = 0xdede0000;
                SD_CMD_RSP_Buff[1] = 0xdede0100;
                SD_CMD_RSP_Buff[2] = 0xdede005a;
            }

            uc_mem_write(MTK, 0x74005200, SD_CMD_RSP_Buff, 32);
        sd_cmd_done:
            FICE_Status = 0xffff;
            EnqueueVMEvent(VM_EVENT_MSDC_IRQ, 0, 0);
        }
        break;
    case 0x74005048:
        if (type == UC_MEM_READ)
        {
            value = 0x100;
            uc_mem_write(MTK, (u32)address, &value, 4);
        }
        break;
    case 0x740031A8:
        if (type == UC_MEM_READ)
        {
            uc_mem_read(MTK, 0x74003000, &value, 4);
            if ((value & 2) != 0)
            {
                value |= 384;
            }
            else
            {
                value |= 384;
            }
            uc_mem_write(MTK, (u32)address, &value, 4);
        }
        break;
    case 0x74004000:
        if (type == UC_MEM_READ)
        {
            printf("set dma addr:%x\n", value);
        }
        break;
    case 0x74000408:
        if (type == UC_MEM_WRITE)
        {
        }
        else
        {
            value = (1 << 12) | 2;
            uc_mem_write(MTK, (u32)address, &value, 4);
        }
        break;
    case 0x34001854:
        if (type == UC_MEM_WRITE)
        {
            value = 0x1fc;
            uc_mem_write(MTK, 0x34001840, &value, 4);
        }
        break;
    case 0x34001858:
        if (type == UC_MEM_WRITE)
        {
            value = 0x1fc;
            uc_mem_write(MTK, 0x34001840, &value, 4);
        }
        break;
    case 0x3400181C:
        if (type == UC_MEM_WRITE)
        {
            IRQ_MASK_SET_L_Data = value;
        }
        break;
    case 0x34001820:
        if (type == UC_MEM_WRITE)
        {
            IRQ_MASK_SET_H_Data = value;
        }
        break;
    case 0x3400406C:
        if (type == UC_MEM_WRITE)
        {
            if (value & 1)
            {
                value = 2 | 4 | 8;
                uc_mem_write(MTK, 0x34004070, &value, 4);
            }
        }
        break;
    case 0x74005118:
        if (type == UC_MEM_READ)
        {
            tmp = 0x40 | 0x80;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x74005040:
        if (type == UC_MEM_WRITE)
        {
            SD_CMD_Buff_Idx = 0;
            FICE_Status = 0;
        }
        break;
    case 0xD08DA8:
        if (type == UC_MEM_READ)
        {
            value = 2;
            uc_mem_write(MTK, (u32)address, &value, 4);
        }
        break;
    case 0xd0291b:
        if (type == UC_MEM_READ)
        {
            value = 5;
            uc_mem_write(MTK, (u32)address, &value, 4);
        }
        break;
    /* 未在 switch 中逐例列出的地址：触摸 AUXADC 窗口、GPT、MSDC 等 */
    default:
        if (address >= 0x3400C080u && address < 0x3400C400u)
        {
            if (type == UC_MEM_WRITE && address == 0x3400C184u)
                auxadc_last_cmd = (u32)value;
            else if (type == UC_MEM_READ && address == 0x3400C198u)
            {
                tmp = auxadc_get_result();
                if (size >= 4)
                    uc_mem_write(MTK, (u32)address, &tmp, 4);
                else if (size == 2)
                {
                    u16 half = (u16)(tmp & 0xFFFFu);
                    uc_mem_write(MTK, (u32)address, &half, 2);
                }
                else if (size == 1)
                {
                    u8 b = (u8)(tmp & 0xFFu);
                    uc_mem_write(MTK, (u32)address, &b, 1);
                }
            }
            else if (type == UC_MEM_READ)
            {
                mtk_touch_hook_mem_read(address);
            }
        }
        else if (address >= 0x81060000 && address < 0x81060100)
            handleGptReg(address, data, value);
        else if ((address >= 0x810E0000u && address <= 0x810E00FFu) ||
                 (address >= 0x81020200u && address <= 0x810202FFu))
        {
            u32 is_wr = (type == UC_MEM_WRITE) ? 1u : 0u;
            handleMsdcReg(address, is_wr, (uint64_t)(uint32_t)value);
        }
        break;
    }
}
void hookRamErrorBack(uc_engine *uc, uc_mem_type type, uint64_t address, uint32_t size, int64_t value, u32 data)
{
    printf("地址无法访问:%x\n", address);
}

bool hookInsnInvalid(uc_engine *uc, void *user_data)
{
    u32 insn;
    u32 pc;
    u32 cpsr;

    (void)uc;
    (void)user_data;

    uc_reg_read(MTK, UC_ARM_REG_PC, &pc);
    uc_mem_read(MTK, pc, &insn, 4);

    if (pc == 0x7C322C || pc == 0x7C3238)
    {
#if MORAL_LOG_UC_CODE_PATCH
        printf("mrc指令:%x\n", insn);
#endif
        return 0;
    }

    uc_reg_read(MTK, UC_ARM_REG_CPSR, &cpsr);
    if ((cpsr & 0x20u) == 0u)
    {
        u32 nc = cpsr | 0x20u;
        uc_reg_write(MTK, UC_ARM_REG_CPSR, &nc);
#if MORAL_LOG_UC_CODE_PATCH
        {
            static u32 arm_thumb_fix_log = 32u;
            if (arm_thumb_fix_log > 0u)
            {
                arm_thumb_fix_log--;
                printf("[UC] INSN_INVALID: T=0→1 继续 PC=%08x insn=%08x (mode=%02x)\n",
                       pc, insn, cpsr & 0x1fu);
            }
        }
#endif
        return 1;
    }

#if MORAL_LOG_UC_CODE_PATCH
    printf("指令无效:%x\n", insn);
#endif
    return 0;
}