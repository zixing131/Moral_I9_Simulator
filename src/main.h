#pragma once

#include "typedef.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "../Lib/sdl2-2.0.10/include/SDL2/SDL.h"
#include "../Lib/unicorn-2.1.4/unicorn/unicorn.h"
#include <pthread.h>
#include "defined.h"
#include "config.h"
#include "keypad.h"

/**
 * 按 Surface 实际 BytesPerPixel 写像素（勿假定 32 位 + x*4，否则会整屏竖条/花屏）。
 * c 须为 SDL_MapRGB(s->format, ...) 对该 surface 编码后的值。
 */
static inline void SDL_PutPixel32(SDL_Surface *s, unsigned x, unsigned y, Uint32 c)
{
    Uint8 *p = (Uint8 *)s->pixels + y * (unsigned)s->pitch + x * (unsigned)s->format->BytesPerPixel;
    switch (s->format->BytesPerPixel)
    {
    case 4:
        *(Uint32 *)p = c;
        break;
    case 3:
#if SDL_BYTEORDER == SDL_BIG_ENDIAN
        p[0] = (Uint8)((c >> 16) & 0xff);
        p[1] = (Uint8)((c >> 8) & 0xff);
        p[2] = (Uint8)(c & 0xff);
#else
        p[0] = (Uint8)(c & 0xff);
        p[1] = (Uint8)((c >> 8) & 0xff);
        p[2] = (Uint8)((c >> 16) & 0xff);
#endif
        break;
    case 2:
        *(Uint16 *)p = (Uint16)c;
        break;
    default:
        *(Uint32 *)p = c;
        break;
    }
}

/**
 * 定义
 * 0-1024 为栈空间
 * 1024-4096为代码空间
 */
char *getRealMemPtr(u32 ptr);
void SimulatePressKey(u8, u8);
void RunArmProgram(void *);
void hookBlockCallBack(uc_engine *uc, uint64_t address, uint32_t size, void *user_data);
void hookCodeCallBack(uc_engine *uc, uint64_t address, uint32_t size, void *user_data);
extern u8 mssend_pop_log;
extern u8 de_small_pending;
extern u32 adc_snapshot_x;
extern u32 adc_snapshot_y;
void hookRamCallBack(uc_engine *uc, uc_mem_type type, uint64_t address, uint32_t size, int64_t value, u32 data);
void onCPRSChange(uc_engine *uc, uint64_t address, uint32_t size, u32 data);
void SaveCpuContext(u32 *);
void RestoreCpuContext(u32 *stackPtr);
void renderGdiBufferToWindow(void);
void Update_RTC_Time(void);
int utf16_len(char *utf16);
bool writeSDFile(u8 *Buffer, u32 startPos, u32 size);
u8 *readSDFile(u32 startPos, u32 size);
void saveFlashFile();
void readFlashFile();
void StartCallback(u32 callbackAddr, u32 r0);
bool StartInterrupt(u32, u32);
void handleEvent_EMU(uc_engine *uc, uint64_t address, uint32_t size, void *user_data);
void LcdUpdateTask(void);
bool isIRQ_Disable(u32 cpsr);
bool isIrqMode(u32 cpsr);

#define NC_BASE 0x74005000
#define NC_MIE_EVENT (NC_BASE + 0x0)
#define NC_MIE_PRIORITY (NC_BASE + 0x8) // 0x20表示等待fifo ready
#define NC_MIE_PATH (NC_BASE + 0x28)
#define NC_AUXREG_ADR (NC_BASE + 0x10C)
#define NC_AUXREG_DAT (NC_BASE + 0x110)
#define NC_CTRL (NC_BASE + 0x114)
#define FCIE_NC_RBUF_CIFD_BASE (NC_BASE + 0x400)

u32 halTimerCount = 0;
u32 halTimerOutLength = 0;
u32 FICE_Status = 0;
u32 IRQ_MASK_SET_L_Data;
u32 IRQ_MASK_SET_H_Data;

u8 *ROM_MEMPOOL;
u8 *ROM3_MEMPOOL;
u8 *ROM2_MEMPOOL;
u8 *RAM_MEMPOOL;
u8 *RAM40_POOL;
u8 *RAMF0_POOL;
u8 *RAM82000000_POOL;

u32 dspDafFlag;
u32 dspDaf_EndingState;
u32 dspSynFlag;

u32 NorFlashID = 0;
u32 MPU_Setting = 0;
u32 MPU_Setting_ROM_Addr = 0;

uc_engine *MTK;
u8 isBreakPointHit = 0;
u32 changeTmp = 0;
u32 changeTmp1 = 0;
u32 changeTmp2 = 0;
u32 changeTmp3 = 0;
u32 irq_sp;
SDL_Window *window;
u32 lastAddress = 1;
u32 systemTick = 0;
u32 lastmemcp;

u32 systemTickReg;
u8 globalSprintfBuff[256] = {0};
u8 sprintfBuff[256] = {0};
u32 dividend; // 被除数
u32 divisor;  // 除数
u32 divideControl;
u32 fcch_sch = 0;
int irq_nested_count;
u32 irq_stack_ptr;
u32 debugType;

clock_t currentTime = 0;
long frameTicks = 0;

int regs[] = {UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_R4, UC_ARM_REG_R5, UC_ARM_REG_R6, UC_ARM_REG_R7, UC_ARM_REG_R8,
              UC_ARM_REG_R9, UC_ARM_REG_R10, UC_ARM_REG_R11, UC_ARM_REG_R12, UC_ARM_REG_R13, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_CPSR};

u32 HalVMMPControl;

typedef struct _nandPage512
{
    u8 pageBuff[512];
    u16 spareBuff[8];
} nandPage512;
typedef struct _nandPage2048
{
    u8 pageBuff[2048];
    u16 spareBuff[32];
} nandPage2048;
// 256MB
u8 NandFlashCard[1024 * 1024 * 256];
// 预留32kb空间，防止连续page读写出错
u32 nandSpareBuff[1024 * 32];

u32 nandDmaBuffPtr;
u32 nandIdInfo[16] = {0x98, 0xa1, 0x90, 0x15, 0x76, 0x14};

typedef struct VM_DMA_CONFIG_
{
    int control;
    DMA_MASTER_CHANEL chanel;
    DMA_DATA_DIRECTION direction;
    DMA_DATA_BYTE_ALIGN align;
    int data_addr;
    int transfer_count;
    int config_finish;
    int alert_length;
    int transfer_end_interrupt_enable;
    u8 cacheBuffer[4096];
    u32 MSDC_DATA_ADDR;
} VM_DMA_CONFIG;

u32 Lcd_Buffer_Ptr;
u32 Lcd_FullScreen_Ptr;
u32 Lcd_Update_X;
u32 Lcd_Update_Y;
u32 Lcd_Update_W;
u32 Lcd_Update_H;
u32 Lcd_Update_Pitch;
u8 Lcd_Cache_Buffer[240 * 400 * 4];
u8 Lcd_Periodic_Buffer[240 * 400 * 4];
u8 Lcd_Need_Update = 0;
/** 由 0x7400313C 首次成功刷屏后置 1；此前禁止周期性显存拉取，避免开机动画/未就绪缓冲花屏 */
u8 De_PeriodicRefreshAllowed;
/** DE trigger 触发时记录时间，周期性刷新在近期有 DE 活动时跳过，避免覆盖新渲染的内容 */
clock_t De_LastTriggerTime;

u32 DE_Layer0_Ptr;
u32 DE_Layer0_W;
u32 DE_Layer0_H;
u32 DE_Layer0_Pitch;

void my_memcpy(void *dest, void *src, int len)
{
    memcpy(dest, src, len);
}

void my_memset(void *dest, char value, int len)
{
    memset(dest, value, len);
}

/**
 * 内存比较 1 相等 0 不相等
 */
u8 my_mem_compare(u8 *src, u8 *dest, u32 len)
{
    while (*src++ == *dest++)
    {
        len--;
        if (len == 0)
            return 1;
    }
    return 0;
}