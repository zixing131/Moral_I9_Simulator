#include "main.h"
#include "lcd.h"
#include "touchscreen.h"
#include <string.h>

/*
 * hookRam.c：Guest 对 MMIO 的读写在 Unicorn 里走 UC_HOOK_MEM_READ/WRITE，由此回调模拟硬件。
 * 含 SPI、定时器、显示引擎 DE、FCIE(NAND/SD)、UART、中断控制器、触摸 AUXADC、GPT、MSDC 等。
 * 与 main.c 的 UC_HOOK_CODE 分工：代码钩子改执行流/寄存器，本文件只伪造寄存器值与 DMA 数据搬运。
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

    de_blit_rgb565_to_surface(sfc, Lcd_Cache_Buffer, dstX, dstY, w, h, cachePitch);
}

static void de_resolve_blit_dest(u32 srcBuf, u16 *dstX, u16 *dstY)
{
    u16 x = (u16)Lcd_Update_X;
    u16 y = (u16)Lcd_Update_Y;

    if (Lcd_FullScreen_Ptr != 0u && srcBuf >= Lcd_FullScreen_Ptr)
    {
        u32 fbBytes = (u32)DE_PANEL_W * (u32)DE_PANEL_H * DE_BPP;
        if (srcBuf < Lcd_FullScreen_Ptr + fbBytes)
        {
            u32 off = srcBuf - Lcd_FullScreen_Ptr;
            u32 mainPitch = (u32)DE_PANEL_W * DE_BPP;
            if (DE_Layer0_Pitch >= mainPitch && DE_Layer0_Pitch < 8192u)
                mainPitch = DE_Layer0_Pitch;
            y = (u16)(off / mainPitch);
            x = (u16)((off % mainPitch) / DE_BPP);
            if (y >= DE_PANEL_H || x >= DE_PANEL_W)
            {
                x = (u16)Lcd_Update_X;
                y = (u16)Lcd_Update_Y;
            }
        }
    }
    *dstX = x;
    *dstY = y;
}

static int de_read_fb(u32 srcAddr, u32 guestPitch, u16 w, u16 h)
{
    u32 rowBytes = (u32)w * DE_BPP;
    if (rowBytes == 0 || h == 0)
        return -1;
    if ((u32)h * rowBytes > sizeof(Lcd_Cache_Buffer))
        return -1;

    if (guestPitch == rowBytes)
    {
        if (uc_mem_read(MTK, srcAddr, Lcd_Cache_Buffer, rowBytes * h) != UC_ERR_OK)
            return -1;
    }
    else
    {
        for (u16 y = 0; y < h; y++)
        {
            u32 readW = (guestPitch < rowBytes) ? guestPitch : rowBytes;
            u8 *dst = Lcd_Cache_Buffer + (u32)y * rowBytes;
            if (uc_mem_read(MTK, srcAddr + (u32)y * guestPitch, dst, readW) != UC_ERR_OK)
                return -1;
            if (readW < rowBytes)
                memset(dst + readW, 0, rowBytes - readW);
        }
    }
    return 0;
}

static u32 de_periodic_call_cnt = 0;

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

static u8 de_deferred_pending = 0;
static u32 de_deferred_src_buf = 0;
static u32 de_deferred_pitch = 0;
static u16 de_deferred_w = 0;
static u16 de_deferred_h = 0;
static uint64_t de_deferred_last_draw = 0;
static uint64_t lcd_irq_last_enqueue = 0;

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

static void de_schedule_deferred_refresh(u32 srcBuf, u32 pitch, u16 w, u16 h)
{
    de_deferred_pending = 1;
    de_deferred_src_buf = srcBuf;
    de_deferred_pitch = pitch;
    de_deferred_w = w;
    de_deferred_h = h;
    De_LastTriggerTime = moral_get_ticks_ms();

    Lcd_FullScreen_Ptr = srcBuf;
    De_PeriodicRefreshAllowed = 1;
}

void de_emulator_flush_pending(void)
{
    u32 srcBuf;
    u32 pitch;
    u16 w;
    u16 h;
    uint64_t now;
    uint64_t min_interval = 1000 / 60;

    if (!de_deferred_pending)
        return;

    now = moral_get_ticks_ms();
    if (min_interval < 1)
        min_interval = 1;
    if (de_deferred_last_draw != 0 &&
        (now - de_deferred_last_draw) < min_interval)
    {
        
        de_deferred_pending = 0;
        return;
    }

    srcBuf = de_deferred_src_buf;
    pitch = de_deferred_pitch;
    w = de_deferred_w;
    h = de_deferred_h;

    if (Lcd_FullScreen_Ptr >= 0x1000u && Lcd_FullScreen_Ptr < 0x8000000u &&
        (w < (DE_PANEL_W - 20u) || h < (DE_PANEL_H - 80u)))
    {
        srcBuf = Lcd_FullScreen_Ptr;
        pitch = DE_PANEL_W * DE_BPP;
        w = (u16)DE_PANEL_W;
        h = (u16)DE_PANEL_H;
    }

    if (srcBuf < 0x1000u || srcBuf >= 0x8000000u || w == 0 || h == 0)
    {
        de_deferred_pending = 0;
        return;
    }
    if (pitch == 0u)
        pitch = (u32)w * DE_BPP;

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
            return;
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
}

void de_emulator_periodic_refresh(void)
{
    if (!De_PeriodicRefreshAllowed)
        return;

    uint64_t now = moral_get_ticks_ms();
    uint64_t idle_threshold = 100;
    if (De_LastTriggerTime != 0 && (now - De_LastTriggerTime) < idle_threshold)
        return;

    u32 srcBuf = Lcd_FullScreen_Ptr;
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

    u32 pitch = DE_PANEL_W * DE_BPP;

    if (de_read_fb_to(Lcd_Periodic_Buffer, sizeof(Lcd_Periodic_Buffer),
                      srcBuf, pitch, DE_PANEL_W, DE_PANEL_H) != 0)
        return;

    de_blit_from(Lcd_Periodic_Buffer, 0, 0, DE_PANEL_W, DE_PANEL_H, DE_PANEL_W * DE_BPP);
    Lcd_Need_Update = 1;
    Perf_LcdRefreshCount++;
    de_periodic_call_cnt++;
    if (MORAL_LOG_HOT_PATH && (de_periodic_call_cnt <= 5 || (de_periodic_call_cnt % 200) == 0))
        printf("[LCD-REFRESH] periodic #%u buf=0x%x\n", de_periodic_call_cnt, srcBuf);
}

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
            if (MORAL_LOG_HOT_PATH && (layer_sel_cnt <= 10 || (layer_sel_cnt % 500) == 0))
                printf("[lcd]layer_sel[%x] #%u\n", (u32)value, layer_sel_cnt);
        }
        break;
    case 0x74003040:
        if (type == UC_MEM_WRITE)
        {
            Lcd_Buffer_Ptr &= 0xffff0000;
            Lcd_Buffer_Ptr |= (value & 0xffff);
            De_PeriodicRefreshAllowed = 1;
            Lcd_FullScreen_Ptr = Lcd_Buffer_Ptr;
        }
        break;
    case 0x74003044:
        if (type == UC_MEM_WRITE)
        {
            Lcd_Buffer_Ptr &= 0x0000ffff;
            Lcd_Buffer_Ptr |= ((value & 0xffff) << 16);
        }
        break;
    case 0x74003054:
        if (type == UC_MEM_WRITE)
        {
            DE_Layer0_Ptr &= 0xffff0000;
            DE_Layer0_Ptr |= (value & 0xffff);
        }
        break;
    case 0x74003058:
        if (type == UC_MEM_WRITE)
        {
            DE_Layer0_Ptr &= 0x0000ffff;
            DE_Layer0_Ptr |= ((value & 0xffff) << 16);
        }
        break;
    case 0x7400305c:
        if (type == UC_MEM_WRITE)
            DE_Layer0_W = value;
        break;
    case 0x74003060:
        if (type == UC_MEM_WRITE)
            DE_Layer0_H = value;
        break;
    case 0x74003064:
        if (type == UC_MEM_WRITE)
            DE_Layer0_Pitch = value;
        break;
    case 0x7400313C:
        if (type == UC_MEM_WRITE)
        {
            u32 srcBuf = Lcd_Buffer_Ptr;
            u32 pitch = Lcd_Update_Pitch;
            u16 w = (u16)Lcd_Update_W;
            u16 h = (u16)Lcd_Update_H;

            static u32 de_trigger_cnt = 0;
            static u32 boot_desc_buf = 0;
            de_trigger_cnt++;

            if (w == 0 || h == 0 || srcBuf < 0x1000u || srcBuf >= 0x8000000u)
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

            if (MORAL_LOG_HOT_PATH && (de_trigger_cnt <= 20 || (de_trigger_cnt % 100) == 0))
            {
                printf("[DE-trigger] #%u buf=0x%x pitch=%u w=%u h=%u\n",
                       de_trigger_cnt, srcBuf, pitch, w, h);
            }

            u8 is_cursor_de = (w <= 20u && h <= 20u);

            if (is_cursor_de)
            {
                uint64_t now = moral_get_ticks_ms();
                uint64_t min_interval = 1000 / 60;
                if (min_interval < 1)
                    min_interval = 1;

                if (cursor_de_last_draw == 0 ||
                    (now - cursor_de_last_draw) >= min_interval)
                {
                    cursor_de_last_draw = now;
                    if (de_read_fb(srcBuf, pitch, w, h) == 0)
                    {
                        u16 dstX, dstY;
                        de_resolve_blit_dest(srcBuf, &dstX, &dstY);
                        de_blit_to_sdl(dstX, dstY, w, h, (u32)w * DE_BPP);
                        Lcd_Need_Update = 1;
                        Perf_LcdRefreshCount++;
                    }
                }

                tmp = 0xF00;
                uc_mem_write(MTK, 0x74003148u, &tmp, 4);
            }
            else
            {
                de_schedule_deferred_refresh(srcBuf, pitch, w, h);
                tmp = 0xF00;
                uc_mem_write(MTK, 0x74003148u, &tmp, 4);
                lcd_irq_enqueue_throttled();
            }
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
                u32 srcBuf = Lcd_Buffer_Ptr;
                Lcd_FullScreen_Ptr = srcBuf;
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

    case 0x74003048:
        if (type == UC_MEM_WRITE)
        {
            Lcd_Update_W = value;
        }
        break;
    case 0x7400304C:
        if (type == UC_MEM_WRITE)
        {
            Lcd_Update_H = value;
        }
        break;
    case 0x74003050:
        if (type == UC_MEM_WRITE)
        {
            Lcd_Update_Pitch = value;
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