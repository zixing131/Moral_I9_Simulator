#include "main.h"
#include "lcd.h"
#include "touchscreen.h"
#include <string.h>

bool hookInsnInvalid(uc_engine *uc, void *user_data);
void hookRamCallBack(uc_engine *uc, uc_mem_type type, uint64_t address, uint32_t size, int64_t value, u32 data);
void handleLcdReg(uint64_t address, u32 data, uint64_t value);
void handleTouchScreenReg(uint64_t address, u32 data, uint64_t value);
void handleGptReg(uint64_t address, u32 data, uint64_t value);

u32 halTimerCnt = 0;              // HalTimerUDelay调用
u32 halTimerCntMax = 0;           // HalTimerUDelay调用
u32 DrvTimerStdaTimerGetTick = 0; // DrvTimerStdaTimerGetTick

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

u32 SD_CMD_Buff[8];
u16 SD_CMD_Buff_Idx = 0;
u32 nandFlashCMD = 0;
u32 nandFlashRow = 0;

u32 nandFlashCmdIdx = 0;
u16 nandFlashCMDData[32];

u32 nandFlashSectorSize = 0;
u32 nandParamSect = 0;
u32 LCD_CMD_Data = 0;

/*
 * AUX ADC 触摸采样模拟。
 * 固件通过写 0x3400C184 选择 ADC 通道，再读 0x3400C198 取 10-bit 结果。
 * 通道命令：0x711 → X，0xB81 → Y，其余 → 压力值
 */
static u32 auxadc_last_cmd = 0;

static u8 auxadc_log_count = 0;

/* 固件写入值可能在不同位域带通道号，取多种切片与已知通道表匹配 */
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
    case 0x711: /* X 坐标通道 */
        result = (touchX < 240u ? touchX : 239u) * 1023u / 240u;
        break;
    case 0xB81: /* Y 坐标通道 */
        result = (touchY < 400u ? touchY : 399u) * 1023u / 400u;
        break;
    case 0x781: /* Z1 压力通道 — 未触摸时返回 0（电路开路无电流），
                  固件 MdlTouchScreenHandle 的 LABEL_13 据此生成 pen-up */
        result = isTouchDown ? 200u : 0u;
        break;
    case 0x7B1: /* Z2 压力通道 */
        result = isTouchDown ? 600u : 1023u;
        break;
    case 0x181: /* X 电阻测量通道 (gnXResistance) */
        result = isTouchDown ? 300u : 0u;
        break;
    case 0x191: /* Y 电阻测量通道 (gYResistance) */
        result = isTouchDown ? 300u : 0u;
        break;
    default:
        /* 未知通道：有按压时给非饱和值，避免驱动认为开路 */
        result = isTouchDown ? 400u : 0u;
        break;
    }
    if (auxadc_log_count < 48)
    {
        auxadc_log_count++;
        printf("[ADC-read] raw_cmd=0x%x eff_ch=0x%x result=%u down=%d x=%u y=%u\n",
               auxadc_last_cmd, cmd12, result, isTouchDown, touchX, touchY);
    }
    return result;
}

#define DE_PANEL_W 240u
#define DE_PANEL_H 400u
#define DE_BPP     2u

/*
 * 简单直接：从 Lcd_Buffer_Ptr 按 Lcd_Update_Pitch（字节行距）整块或逐行读到 Lcd_Cache_Buffer，
 * 然后 blit 到 SDL。不再试图解析 DE Layer0 描述符（此固件下始终返回垃圾值）。
 */
static u8 de_blit_logged = 0;

static void de_blit_to_sdl(u16 dstX, u16 dstY, u16 w, u16 h, u32 cachePitch)
{
    SDL_Surface *sfc = SDL_GetWindowSurface(window);
    if (!sfc)
        return;

    if (!de_blit_logged)
    {
        de_blit_logged = 1;
        printf("[SDL-surface] w=%d h=%d pitch=%d bpp=%d Rmask=0x%x\n",
               sfc->w, sfc->h, sfc->pitch, sfc->format->BytesPerPixel, sfc->format->Rmask);
    }

    for (u16 yi = 0; yi < h && (dstY + yi) < DE_PANEL_H; yi++)
    {
        u8 *row = Lcd_Cache_Buffer + cachePitch * (u32)yi;
        for (u16 xi = 0; xi < w && (dstX + xi) < DE_PANEL_W; xi++)
        {
            u16 color = *((u16 *)row + xi);
            u8 r = (u8)PIXEL565R(color);
            u8 g = (u8)PIXEL565G(color);
            u8 b = (u8)PIXEL565B(color);
            SDL_PutPixel32(sfc, dstX + xi, dstY + yi, SDL_MapRGB(sfc->format, r, g, b));
        }
    }
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

void de_emulator_periodic_refresh(void)
{
    if (!De_PeriodicRefreshAllowed)
        return;

    u32 srcBuf = Lcd_FullScreen_Ptr;
    if (srcBuf < 0x1000u || srcBuf >= 0x8000000u)
        srcBuf = Lcd_Buffer_Ptr;
    if (srcBuf < 0x1000u || srcBuf >= 0x8000000u)
        return;

    u32 pitch = DE_PANEL_W * DE_BPP;

    if (de_read_fb(srcBuf, pitch, DE_PANEL_W, DE_PANEL_H) != 0)
        return;

    de_blit_to_sdl(0, 0, DE_PANEL_W, DE_PANEL_H, DE_PANEL_W * DE_BPP);
    Lcd_Need_Update = 1;
    de_periodic_call_cnt++;
    if (de_periodic_call_cnt <= 5 || (de_periodic_call_cnt % 200) == 0)
        printf("[LCD-REFRESH] periodic #%u buf=0x%x\n", de_periodic_call_cnt, srcBuf);
}

void hookRamCallBack(uc_engine *uc, uc_mem_type type, uint64_t address, uint32_t size, int64_t value, u32 data)
{

    u32 tmp;
    u32 *ptr1;
    switch (address)
    {
    case 0x74006CA8: //
        if (type == UC_MEM_WRITE)
        {
            if (value == 0x3333)
            { // 写0x3333 开始SPI数据交互
                SPI_CMD_Buf_Idx = 0;
            }
            else if (value == 0x5555)
            { // 写0x5555 结束SPI数据交互

                SPI_CMD_Buf_Idx = 0;
            }
        }
        break;
    case 0x74006C10: // HalPagingSpiBusWrite SPI发送数据状态寄存器
        if (type == UC_MEM_WRITE)
        {
            SPI_CMD_Buf[SPI_CMD_Buf_Idx] = (u8)value;
            SPI_CMD_Buf_Idx++;
        }
        break;
    case 0x74006C14: // HalPagingSpiBusRead SPI总线接收数据寄存器
        if (type == UC_MEM_READ)
        {
            if (SPI_CMD_Buf_Idx > 0) // 开始处理命令
            {
                // printf("HalPagingSpiBusWrite:(%d)", SPI_CMD_Buf_Idx);
                // for (tmp = 0; tmp < SPI_CMD_Buf_Idx; tmp++)
                // {
                //     printf("%02x", SPI_CMD_Buf[tmp]);
                // }
                // printf("\n");
                if (SPI_CMD_Buf_Idx == 3)
                { // gCmdWaitRdy__HalAsuraWaitRdy
                    // printf("[SPI]gCmdWaitRdy__HalAsuraWaitRdy\n");
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
                if (SPI_CMD_Buf_Idx == 5 && SPI_CMD_Buf[0] == 0x15)
                { // 写数据
                  // u16 *p = &SPI_CMD_Buf[1];
                  // u16 addr = *p;
                  // p = &SPI_CMD_Buf[3];
                  // u16 data = *p;
                  // printf("[SPI] Addr:%x", addr);
                  // printf(" Value:%x\n", data);
                }
            }
            if (SPI_CMD_Buf_Idx < 1)
            {
                printf("error HalPagingSpiBusWrite\n");
                while (1)
                    ;
            }
            uc_mem_write(MTK, (u32)address, &SPI_DATA_Buf[--SPI_DATA_Buf_Idx], 4);
        }
        break;
    case 0x74006C30: // HalPagingSpiBusRead 写1开始接收数据
        break;
    case 0x74006C54: // HalPagingSpiBusRead SPI接收状态寄存器 是否完成接收1字节
        if (type == UC_MEM_READ)
        {
            tmp = 1;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x74006C58: // HalPagingSpiBusWrite SPI发送状态寄存器 是否完成发送1字节
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
    case 0x74006480: // HalPllAbbSetSpeed
        if (type == UC_MEM_READ)
        {
            tmp = 2;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x34002C58: // DrvTimerGlobalTimerGetTick
    case 0x34002C44:
        if (type == UC_MEM_READ)
        {
            DrvTimerStdaTimerGetTick += 1;
            uc_mem_write(MTK, (u32)address, &DrvTimerStdaTimerGetTick, 4);
        }

        break;
    case 0x3400AD14: // HalTimerUDelay
        if (type == UC_MEM_READ)
        {
            value = (1 << 9);
            uc_mem_write(MTK, (u32)address, &value, 4);
        }
        break;
    // case 0x34002C08: // DrvTimerOstickGetCount
    //     if (type == UC_MEM_WRITE)
    //     {
    //         halTimerCount = value;
    //         printf("write tickCount\n");
    //         while(1);
    //     }
    //     break;
    case 0x34002c00: // 定时器计数,最大2047,请勿继续改动这里
        if (type == UC_MEM_READ)
        {
            halTimerCnt = halTimerCnt == 0 ? 2047 : 0;
            uc_mem_write(MTK, (u32)address, &halTimerCnt, 4);
        }
        break;
    // case 0x740031C0:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         printf("[lcd]31c0[%x]\n", value);
    //         uc_mem_read(MTK, Lcd_Buffer_Ptr, Lcd_Cache_Buffer, Lcd_Update_W * Lcd_Update_H * 2);
    //         Lcd_Need_Update = 1;
    //         while (Lcd_Need_Update == 1)
    //             ;
    //     }
    //     break;
    // case 0x74003094:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         printf("[lcd]buff_format[%x]\n", value);
    //     }
    //     break;
    case 0x7400309C:
        if (type == UC_MEM_WRITE)
        {
            printf("[lcd]layer_sel[%x]\n", value);
        }
        break;
    case 0x74003040:
        if (type == UC_MEM_WRITE)
        {
            Lcd_Buffer_Ptr &= 0xffff0000;
            Lcd_Buffer_Ptr |= (value & 0xffff);
            if (Lcd_Update_W >= DE_PANEL_W && Lcd_Update_H >= DE_PANEL_H &&
                Lcd_Buffer_Ptr >= 0x1000u && Lcd_Buffer_Ptr < 0x8000000u)
            {
                if (Lcd_FullScreen_Ptr != 0 && Lcd_FullScreen_Ptr != Lcd_Buffer_Ptr)
                    De_PeriodicRefreshAllowed = 1;
                Lcd_FullScreen_Ptr = Lcd_Buffer_Ptr;
            }
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
        // case 0x74003100:
        //     if (type == UC_MEM_WRITE)
        //     {
        //     }
        //     break;
    case 0x7400313C:
        if (type == UC_MEM_WRITE)
        {
            u32 srcBuf = Lcd_Buffer_Ptr;
            u32 pitch = Lcd_Update_Pitch;
            u16 w = (u16)Lcd_Update_W;
            u16 h = (u16)Lcd_Update_H;

            if (w == 0 || h == 0 || srcBuf < 0x1000u || srcBuf >= 0x8000000u)
            {
                EnqueueVMEvent(VM_EVENT_LCD_IRQ, 0, 0);
                break;
            }
            if (w > DE_PANEL_W)
                w = (u16)DE_PANEL_W;
            if (h > DE_PANEL_H)
                h = (u16)DE_PANEL_H;
            if (pitch == 0u)
                pitch = (u32)w * DE_BPP;

            static u32 de_trigger_cnt = 0;
            de_trigger_cnt++;
            if (de_trigger_cnt <= 5 || (de_trigger_cnt % 100) == 0)
            {
                printf("[DE-trigger] #%u buf=0x%x pitch=%u w=%u h=%u\n",
                    de_trigger_cnt, srcBuf, pitch, w, h);
            }

            if (de_read_fb(srcBuf, pitch, w, h) == 0)
            {
                de_blit_to_sdl((u16)Lcd_Update_X, (u16)Lcd_Update_Y, w, h, (u32)w * DE_BPP);
                Lcd_Need_Update = 1;
                if (w >= DE_PANEL_W && h >= DE_PANEL_H)
                {
                    /* 首帧全屏 DE 常指向未就绪帧缓冲 → 周期读会花屏；跳过第 1 次再允许周期刷新 */
                    static u8 de_fullscreen_ready_hits;
                    Lcd_FullScreen_Ptr = srcBuf;
                    if (de_fullscreen_ready_hits < 250u)
                        de_fullscreen_ready_hits++;
                    if (de_fullscreen_ready_hits >= 2u)
                        De_PeriodicRefreshAllowed = 1;
                }
            }
            EnqueueVMEvent(VM_EVENT_LCD_IRQ, 0, 0);
        }
        break;
    case 0x740031A0: // HalDispSetCmdPhase
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
            // LCD Pitch
            Lcd_Update_Pitch = value;
        }
        break;

    case 0x34000424: // chipVersion NC_PlatformInit
        if (type == UC_MEM_READ)
        {
            tmp = 3; // 硬件已完善，不需要软件补丁
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
        // case 0x34005000: //  UART1
        //     if (type == UC_MEM_WRITE)
        //     {
        //         UART1_Buf[UART1_Buf_Idx++] = (u8)value;
        //         if (value == 0xd)
        //         {
        //             UART1_Buf[UART1_Buf_Idx++] = 0;
        //             printf("[uart1]%s", UART1_Buf);
        //             UART1_Buf_Idx = 0;
        //         }
        //     }
        //     break;
        // case 0x34005400: // UART2
        //     if (type == UC_MEM_WRITE)
        //     {
        //         UART2_Buf[UART2_Buf_Idx++] = (u8)value;
        //         if (value == 0xd)
        //         {
        //             UART2_Buf[UART2_Buf_Idx++] = 0;
        //             printf("[uart2]%s", UART2_Buf);
        //             UART2_Buf_Idx = 0;
        //         }
        //     }
        //     break;

    case 0x3400500C: // hal_uart_debug_init  UART1
    case 0x3400540C: // UART2
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
    case 0x34002C10: // reset watch dog
        if (type == UC_MEM_WRITE)
        {
            if (value == 0x11FFF || value == 0x10FFF) // HalFcieWaitMieEvent的逻辑
            {
                // 看门狗重置后mie_event重置为0x200??
                FICE_Status = 0x02000200;

                // nandFlashCmdIdx = 0;
                // my_memset(nandFlashCMDData, 0, sizeof(nandFlashCMDData));
            }
            // printf("watch dog reset v:%x\n", value);
        }
        break;
    case NC_MIE_EVENT: // FCIE 状态寄存器
        if (type == UC_MEM_READ)
        {
            // tmp = 0xffff;
            // uc_mem_write(MTK, (u32)address, &tmp, 4);
            // printf("read fcie %x\n", FICE_Status);
            uc_mem_write(MTK, (u32)address, &FICE_Status, 4);
        }
        else
        {
            FICE_Status = value;
            // if ((value & 2) == 2) // 写2清0
            //     FICE_Status = 0;
            // if ((value & 4) == 4) // 写2清0
            //     FICE_Status = 0;
            // if ((value & 6) == 6) // 写2清0
            //     FICE_Status = 0;
            // if ((value & 0x801) == 0x801) // 写2清0
            //     FICE_Status = 0;
        }
        break;
    case 0x74005004: // FCIE 中断状态寄存器 NC_wait_MIULastDone
        if (type == UC_MEM_READ)
        {
            tmp = 0; // 0x80表示中断完成
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x74005008: // nan flash control
        if (type == UC_MEM_READ)
        {
            tmp = 0x20; // fifo ready
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x74005028: // NC_wait_MIULastDone
        if (type == UC_MEM_READ)
        {
            tmp = 0x80; // 表示完成
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    // case 0x74005074:
    // case 0x74005070:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         // printf("Set Miu Addr:%x\n", value);
    //     }
    //     break;
    // case 0x74005034:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         SD_CMD_L = value;
    //         printf("sd cmd low %x\n", value);
    //     }
    //     break;
    // case 0x74005038:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         SD_CMD_H = value;
    //         printf("sd cmd high %x\n", value);
    //     }
    //     break;
    case NC_AUXREG_ADR:
        if (type == UC_MEM_WRITE)
        {
            // 0x19标识读取ID命令
            // 0x20读取Block命令
            nandFlashCMD = value;
            // printf("NC_Config[Column Address]:%x\n", value);
        }
        break;
    case NC_AUXREG_DAT:
        if (type == UC_MEM_WRITE)
        {
            nandFlashCMDData[nandFlashCmdIdx++] = (u16)value;
            if (nandFlashCmdIdx >= 32)
            {
                printf(" %x ", lastAddress);
                printf("write NC_AUXREG_DAT[%d] %x\n", nandFlashCmdIdx, (u16)value);
                while (1)
                    ;
            }
        }
        break;
    case 0x7400515c: // 写入3 开始重新配置NC_Config()
        if (type == UC_MEM_WRITE)
        {
            if (value == 3)
            {
                nandFlashCmdIdx = 0;
                my_memset(nandFlashCMDData, 0, sizeof(nandFlashCMDData));
            }
        }
        break;
    // case 0xdcf420:
    // if (type == UC_MEM_WRITE)
    // {
    //     printf("Write Flag at %x\n",lastAddress);
    // }
    // break;
    case 0x74003124: // LCD命令低16位,请勿继续改动
        if (type == UC_MEM_WRITE)
        {
            LCD_CMD_Data = value;
        }
        break;
    case 0x74003128: // LCD命令高16位,请勿继续改动
        if (type == UC_MEM_WRITE)
        {
            LCD_CMD_Data &= 0xffff;
            LCD_CMD_Data |= (value << 16);
            printf("Handle LCD Cmd %x \n", LCD_CMD_Data);
        }
        break;
    case 0x74003130: // 读取cmd响应
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
            // nandFlashCmdIdx = 0;
            // my_memset(nandFlashCMDData, 0, sizeof(nandFlashCMDData));
            // printf("Write 11c:%x\n", value);
        }

        break;
    // case 0x74003134: // 读取cmd响应

    //     break;
    case NC_CTRL: // 写入3 读取数据传输开始 写入11 写入数据传输开始 写入1 擦除Block开始
        if (type == UC_MEM_WRITE)
        {
            if ((value & 1) == 1)
            {
                // FICE_Status = 512 | 513; // 完成标志
                u16 SectorInPage = nandFlashCMDData[0];
                u32 PhyRowIdx = (nandFlashCMDData[1] | (nandFlashCMDData[2] << 16));
                u32 destRowIdx = (nandFlashCMDData[4] | (nandFlashCMDData[5] << 16));
                u16 OpCode_RW_AdrCycle = nandFlashCMDData[3];
                u32 act = nandFlashCMDData[4] | (nandFlashCMDData[5] << 16);

                // printf("nand>>");
                // printf("[cmd:%08x]", nandFlashCMD);
                // printf("[sec:%08x]", SectorInPage);
                // printf("[pidx:%08x]", PhyRowIdx);
                // printf("[act:%08x]", act);
                // printf("[call:%08x]\n", lastAddress);

                if (nandFlashCMD == 0x20)
                {

                    if (act == 0x88889880) // 读取 sector 数据 nc_read_sectors调用
                    {
                        nandPage512 *pool = (nandPage512 *)NandFlashCard;
                        nandPage512 *p = pool + (PhyRowIdx * 4);
                        uc_mem_write(MTK, nandDmaBuffPtr, p->pageBuff, 512);
                        for (tmp = 0; tmp < 8; tmp++)
                        {
                            nandSpareBuff[tmp] = p->spareBuff[tmp];
                        }
                        uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandSpareBuff, 32);
                        // printf("NC_Read_Sec[%08x][%08x][%08x]%d,%d\n", PhyRowIdx, PhyRowIdx * sizeof(nandPage2048), nandDmaBuffPtr, nandFlashCMDData[6], nandFlashCMDData[7]);
                    }
                    else if (act == 0x88988001) // 读取 page 数据  nc_read_sectors调用
                    {
                        nandPage2048 *p = ((nandPage2048 *)NandFlashCard) + PhyRowIdx;

                        if (nandParamSect == 1)
                        {
                            // 读取page中的sec 0,cnt 1
                            uc_mem_write(MTK, nandDmaBuffPtr, p->pageBuff, 512);
                            for (tmp = 0; tmp < 8; tmp++)
                            {
                                nandSpareBuff[tmp] = p->spareBuff[tmp];
                            }
                            uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandSpareBuff, 32);
                            // printf("NC_Read_Sec0[%08x][%08x][%08x]\n", PhyRowIdx, PhyRowIdx * sizeof(nandPage2048), nandDmaBuffPtr);
                        }
                        else if (nandParamSect == 0x81)
                        {
                            // 读取page中的sec 1,cnt 1
                            uc_mem_write(MTK, nandDmaBuffPtr, p->pageBuff + 512, 512);
                            for (tmp = 0; tmp < 8; tmp++)
                            {
                                nandSpareBuff[tmp] = p->spareBuff[tmp + 8];
                            }
                            uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandSpareBuff, 32);
                            // printf("NC_Read_Sec1[%08x][%08x][%08x]\n", PhyRowIdx, PhyRowIdx * sizeof(nandPage2048), nandDmaBuffPtr);
                        }
                        else if (nandParamSect == 0x101)
                        {
                            // 读取page中的sec 2,cnt 1
                            uc_mem_write(MTK, nandDmaBuffPtr, p->pageBuff + 512 * 2, 512);
                            for (tmp = 0; tmp < 8; tmp++)
                            {
                                nandSpareBuff[tmp] = p->spareBuff[tmp + 16];
                            }
                            uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandSpareBuff, 32);
                            // printf("NC_Read_Sec2[%08x][%08x][%08x]\n", PhyRowIdx, PhyRowIdx * sizeof(nandPage2048), nandDmaBuffPtr);
                        }
                        else if (nandParamSect == 0x181)
                        {
                            // 读取page中的sec 3,cnt 1
                            uc_mem_write(MTK, nandDmaBuffPtr, p->pageBuff + 512 * 3, 512);
                            for (tmp = 0; tmp < 8; tmp++)
                            {
                                nandSpareBuff[tmp] = p->spareBuff[tmp + 24];
                            }
                            uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandSpareBuff, 32);
                            // printf("NC_Read_Sec3[%08x][%08x][%08x]\n", PhyRowIdx, PhyRowIdx * sizeof(nandPage2048), nandDmaBuffPtr);
                        }
                        else if (nandParamSect == 0x85)
                        {
                            // 读取page中的sec 1,cnt 3
                            uc_mem_write(MTK, nandDmaBuffPtr, p->pageBuff + 512, 512 * 3);
                            for (tmp = 0; tmp < 8 * 3; tmp++)
                            {
                                nandSpareBuff[tmp] = p->spareBuff[tmp + 8];
                            }
                            uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandSpareBuff, 32 * 3);
                            // printf("NC_Read_Sec123[%08x][%08x][%08x]\n", PhyRowIdx, PhyRowIdx * sizeof(nandPage2048), nandDmaBuffPtr);
                        }
                        else if (nandParamSect == 0x5)
                        {
                            // 读取page中的sec 0,cnt 3
                            uc_mem_write(MTK, nandDmaBuffPtr, p->pageBuff, 512 * 3);
                            for (tmp = 0; tmp < 8 * 3; tmp++)
                            {
                                nandSpareBuff[tmp] = p->spareBuff[tmp];
                            }
                            uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandSpareBuff, 32 * 3);
                            // printf("NC_Read_Sec012[%08x][%08x][%08x]\n", PhyRowIdx, PhyRowIdx * sizeof(nandPage2048), nandDmaBuffPtr);
                        }
                        else if (nandParamSect == 0x3)
                        {
                            // 读取page中的sec 0,cnt 2
                            uc_mem_write(MTK, nandDmaBuffPtr, p->pageBuff, 512 * 2);
                            for (tmp = 0; tmp < 8 * 2; tmp++)
                            {
                                nandSpareBuff[tmp] = p->spareBuff[tmp];
                            }
                            uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandSpareBuff, 32 * 2);
                            // printf("NC_Read_Sec012[%08x][%08x][%08x]\n", PhyRowIdx, PhyRowIdx * sizeof(nandPage2048), nandDmaBuffPtr);
                        }
                        else
                        {
                            printf("nand>>");
                            printf("[cmd:%08x]", nandFlashCMD);
                            printf("[sec:%08x]", SectorInPage);
                            printf("[pidx:%08x]", PhyRowIdx);
                            printf("[act:%08x]", act);
                            printf("[call:%08x]\n", lastAddress);
                            printf("unhandle nand param sec %x \n", nandParamSect);
                            while (1)
                                ;
                        }
                    }
                    else if (act == 0)
                    {
                        // NC_ResetNandFlash
                        // printf("act 0 call %x\n", lastAddress);
                    }
                    else if (act == 0x8880)
                    {
                        // NC_ProbeReadSeq
                        // printf("act 0x8880 call %x\n", lastAddress);
                    }
                    else if (act == 0x810d800a)
                    {
                        // nand_erase_blk 一个block = 64 page
                        // printf("Start NC_Erase_Blk[%x] \n", PhyRowIdx);
                        // for (u32 cnt = 0; cnt < 32; cnt++)
                        // {
                        //     u32 phyIdx = PhyRowIdx + cnt;
                        //     nandPage2048 *p = ((nandPage2048 *)NandFlashCard) + phyIdx;
                        //     my_memset(p, 0, sizeof(nandPage2048));
                        // }
                    }
                    else
                    {
                        printf("unhandle 0x20 act %x \n", act);
                        printf("%x\n", lastAddress);
                        while (1)
                            ;
                    }
                }
                else if (nandFlashCMD == 0x19) // Read_Id
                {
                    if (act == 0)
                    {
                        uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE, nandIdInfo, 32);
                    }
                    else
                    {
                        printf(" %x ", lastAddress);
                        printf("unhandle 0x19 act %x \n", act);
                        while (1)
                            ;
                    }
                }
                else if (nandFlashCMD == 0x18)
                {
                    int cmd7 = nandFlashCMDData[7];

                    if (act == 0xb0988001) // nc_read_pages调用
                    {
                        u16 pageCnt = nandFlashCMDData[7] + 1;
                        // printf("page cnt: %x\n",pageCnt);
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
                            // printf("NC_Read_P2k[%08x][%08x][%08x]\n", phyIdx, phyIdx * sizeof(nandPage2048), virtAddr);
                        }
                    }
                    else if (act == 0xd800690)
                    {
                        u16 pageCnt = nandFlashCMDData[8] + 1;
                        for (u32 cnt = 0; cnt < pageCnt; cnt++)
                        {
                            u32 phyIdx = PhyRowIdx + cnt;
                            u32 virtAddr = nandDmaBuffPtr + (cnt * 2048);
                            // printf("NC_Write_P2k[%08x][%08x][%08x]\n", phyIdx, phyIdx * sizeof(nandPage2048), virtAddr);
                            nandPage2048 *p = ((nandPage2048 *)NandFlashCard) + phyIdx;
                            uc_mem_read(MTK, virtAddr, p->pageBuff, 2048);
                            // uc_mem_read(MTK, FCIE_NC_RBUF_CIFD_BASE + (cnt * 128), nandSpareBuff, 128);
                            // for (tmp = 0; tmp < 32; tmp++)
                            // {
                            //     p->spareBuff[tmp] = nandSpareBuff[tmp];
                            // }
                        }
                    }
                    else if (cmd7 == 0xa082)
                    {
                        // PageCopy
                        u32 cntAll = 0;
                        uc_mem_read(MTK, 0x74005110, &cntAll, 4);
                        cntAll += 1;
                        // printf("Start Nand_PageCopy %x to ", PhyRowIdx);
                        // printf(" %x ", destRowIdx);
                        // printf(" buff:%x ", nandDmaBuffPtr);
                        // printf(" cnt:%x \n", cntAll);

                        for (u32 cnt = 0; cnt < cntAll; cnt++)
                        {
                            nandPage2048 *src = ((nandPage2048 *)NandFlashCard) + PhyRowIdx + cnt;
                            // uc_mem_write(MTK, FCIE_NC_RBUF_CIFD_BASE + (cnt * 128), src->spareBuff, 128);
                            nandPage2048 *dest = ((nandPage2048 *)NandFlashCard) + destRowIdx + cnt;
                            // uc_mem_write(MTK, nandDmaBuffPtr, src->pageBuff, 2048);
                            // uc_mem_read(MTK, nandDmaBuffPtr + (cnt * 2048), dest->pageBuff, 2048);
                            // uc_mem_read(MTK, FCIE_NC_RBUF_CIFD_BASE + (cnt * 128), dest->spareBuff, 128);
                            my_memcpy(dest, src, sizeof(nandPage2048));
                        }
                    }
                    else
                    {
                        printf("unhandle 0x18 act %x \n", act);
                        printf("%x\n", lastAddress);

                        while (1)
                            ;
                    }
                }
                else
                {
                    printf("unhandle nand cmd %x \n", nandFlashCMD);
                    while (1)
                        ;
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
    // case 0xdcc81c:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         printf("Write EEP ROM at %x\n", lastAddress);
    //     }
    //     break;
    case 0x74005044:
        if (type == UC_MEM_WRITE && value != 0) // 写入0才是执行?
        {
            uc_mem_read(MTK, 0x74005200, &SD_CMD_Buff, 32);
            u32 *p = SD_CMD_Buff;
            // 开始解析SD命令
            if ((p[0] == 0x48) && p[1] == 0x100 && p[2] == 0x5a)
            {
                // SD_CMD_RSP_Buff[0] = 0xdede0000; // 最后两字节有效
                // SD_CMD_RSP_Buff[1] = 0xdede0100;
                // SD_CMD_RSP_Buff[2] = 0xdede005a;

                SD_CMD_RSP_Buff[0] = 0xdede0000; // 最后两字节有效
                SD_CMD_RSP_Buff[1] = 0xdede0100;
                SD_CMD_RSP_Buff[2] = 0xdede005a;

                // printf("hit sd cmd aa \n");
            }
            else if ((p[0] == 0x4069) && p[1] == 0x8001 && p[2] == 0)
            {
                SD_CMD_RSP_Buff[0] = 0xdedec000; // ff为c0时代表sdhc
                SD_CMD_RSP_Buff[1] = 0xdede0000;
                // printf("hit sd cmd ab\n");
            }
            // APP_CMD //SCR  //?? //Set 4bitmode //SetBusWidth
            else if ((p[0] == 0x77 && p[1] == 0) || (p[0] == 0x73 && p[1] == 0) || (p[0] == 0x377 && p[1] == 8) || (p[0] == 0x347 && p[1] == 8) || (p[0] == 0x46 && p[1] == 0 && p[2] == 2))
            {
                // APP_CMD
                SD_CMD_RSP_Buff[0] = 0xdede0000; //[5][4]
                SD_CMD_RSP_Buff[1] = 0xdede0700;
                // printf("hit sd cmd APP_CMD\n");
            }
            else if (p[0] == 0x43 && p[1] == 0)
            {
                // RCA
                SD_CMD_RSP_Buff[0] = 0x01020304;
                SD_CMD_RSP_Buff[1] = 0x05060708;
                // printf("hit sd cmd RCA\n");
            }
            else if (p[0] == 0xff49 && p[1] == 8)
            {
                // CSD
                SD_CMD_RSP_Buff[0] = 0x01020304;
                SD_CMD_RSP_Buff[1] = 0x05060708;
                // printf("hit sd cmd CSD\n");
            }
            else if (p[0] == 0x42 && p[1] == 0)
            {
                // CID
                SD_CMD_RSP_Buff[0] = 0x01020304;
                SD_CMD_RSP_Buff[1] = 0x05060708;
                // printf("hit sd cmd CID\n");
            }
            // else
            //     printf("not hit cmd");
            // printf("[%x]", value);
            // printf(">>handle sd cmd %x", *p++);
            // printf(" %x", *p++);
            // printf(" %x", *p++);
            // printf(" %x", *p++);
            // printf("\n");

            uc_mem_write(MTK, 0x74005200, SD_CMD_RSP_Buff, 32);
            FICE_Status = 0xffff;
            EnqueueVMEvent(VM_EVENT_MSDC_IRQ, 0, 0);
        }
        break;
    case 0x74005048: //_HalFcie_SDWaitD0High
        if (type == UC_MEM_READ)
        {
            value = 0x100;
            uc_mem_write(MTK, (u32)address, &value, 4);
        }
        break;
    case 0x740031A8: // lcd HalDispReadDebugCSValue
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
    // case 0x74005120:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         printf("NC_Config[120]:%x\n", value);
    //     }
    //     break;
    // case 0x74005124:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         printf("NC_Config[124]:%x\n", value);
    //     }
    //     break;
    // case 0x74005140:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         printf("NC_Config[140]:%x\n", value);
    //     }
    //     break;
    // case 0xdd2c80:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         printf("NC_Config SpareBuf: %x \n", lastAddress);
    //     }
    //     break;
    // case 0xdd0c80:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         printf("NC_Config PageBuf: %x \n", lastAddress);
    //     }
    //     break;
    // case FCIE_NC_RBUF_CIFD_BASE: // Nand Flash ID Data?? (共14字节，但是一个字节占用一个寄存器(4字节))
    //     if (type == UC_MEM_WRITE)
    //     {
    //         printf("[nand]write sector data %x\n", lastAddress);
    //     }
    //     break;
    // case 0x3400040C: //_sys_reboot_after_remap
    //     break;
    case 0x34001854: // 中断清除
        if (type == UC_MEM_WRITE)
        {
            IRQ_MASK_SET_L_Data &= (~value);
            uc_mem_write(MTK, 0x3400181C, &IRQ_MASK_SET_L_Data, 4);
            value = 0x1fc; // 设置中断号码大于等于0x40退出中断?
            uc_mem_write(MTK, 0x34001840, &value, 4);
            // printf("清除中断掩码:%x\n", value);
        }
        break;
    case 0x34001858: // 中断清除
        if (type == UC_MEM_WRITE)
        {
            IRQ_MASK_SET_H_Data &= (~value);
            value = 0x1fc; // 设置中断号码大于等于0x40退出中断?
            uc_mem_write(MTK, 0x34001840, &value, 4);
            uc_mem_write(MTK, 0x34001820, &IRQ_MASK_SET_H_Data, 4);
            //            printf("Clear Pending Irq %x\n", value);
        }
        break;
    case 0x3400181C: // HalIntcMask 0-31中断掩码
                     // 0x3400187c
        if (type == UC_MEM_WRITE)
        {
            IRQ_MASK_SET_L_Data |= value;
        }
        break;
    case 0x34001820: // HalIntcMask 32-63中断掩码
                     // 0x34001880
        if (type == UC_MEM_WRITE)
        {
            IRQ_MASK_SET_H_Data |= value;
            // printf("写H中断掩码:%x\n", value);
        }
        break;
    case 0x3400406C: // HalI2cSendDataStandard
        // 0x34004010 i2c数据接收寄存器
        if (type == UC_MEM_WRITE)
        {
            if (value & 1)
            {
                value = 2 | 4 | 8; // 写入2表示发送完毕 写入8表示接收完毕
                uc_mem_write(MTK, 0x34004070, &value, 4);
            }
        }
        break;
    // case 0x3400183C: // 32-63中断号标识位
    // case 0x34001840: // 0-31中断号标识位
    //     break;
    // case 0xdcc7ac:
    //     if (type == UC_MEM_WRITE)
    //     {
    //         printf("写drv nand pageSize:%x\n", value);
    //         printf("call by %x\n", lastAddress);
    //     }
    //     break;
    // case 0x74005100: // NC_ResetFCIE
    //     break;
    case 0x74005118:
        if (type == UC_MEM_READ)
        {
            tmp = 0x40 | 0x80; // 擦除完成状态
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    case 0x74005040:
        if (type == UC_MEM_WRITE)
        {
            SD_CMD_Buff_Idx = 0;
        }
        break;
    case 0xD08DA8: // ConfigRfDone hwlrf_FirstLoadPll
        if (type == UC_MEM_READ)
        {
            // todo 完成RF初始化
            value = 2;
            uc_mem_write(MTK, (u32)address, &value, 4);
        }
        break;
    case 0xd0291b:
        if (type == UC_MEM_READ)
        {
            // todo 完成RF初始化
            value = 5;
            uc_mem_write(MTK, (u32)address, &value, 4);
        }
        break;
    // case 0x34002C04: // 假如设定了超时就重置计数器
    //     if (type == UC_MEM_WRITE)
    //     {
    //         if (value > 0)
    //         {
    //             halTimerOutLength = value;
    //             printf("time out value:%x\n", halTimerOutLength);
    //             halTimerCount = 0;
    //         }
    //     }
    //     break;
    default:
        if (address >= 0x90000000 && address < 0x90001000)
            handleLcdReg(address, data, value);
        else if (address >= 0x82050000 && address < 0x82051000)
            handleTouchScreenReg(address, data, value);
        else if (address >= 0x3400C080u && address < 0x3400C400u)
        {
            /* 触摸 AUXADC：命令可能在 C180~C18C，数据读或为半字/字节 */
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
        else if (address >= 0x74003000 && address < 0x74005000 && type == UC_MEM_WRITE)
        {
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
    u32 tmp;
    u32 Rd;
    u32 CRn;
    u32 CRm;
    u32 insn;
    u32 pc;
    uc_reg_read(MTK, UC_ARM_REG_PC, &pc);
    uc_mem_read(MTK, pc, &insn, 4);
    if (pc == 0x7C322C)
    {
        printf("mrc指令:%x\n", insn);
    }
    else
        printf("指令无效:%x\n", insn);
    // 返回true继续
    // 返回false停止

    return 0;
}