#ifndef LCD_H
#define LCD_H

#include "main.h"
#include "vmEvent.h"

#define LCD_BASE 0x90000000

/** 从模拟器 DE/LCD 缓冲区读取整屏并写入 SDL（供 lcdTaskMain 定时调用） */
void de_emulator_periodic_refresh(void);
void de_emulator_flush_pending(void);
/** 主线程：将模拟线程已拉取到 Lcd_Cache_Buffer 的帧画到 SDL surface（须在 flush 之前调用以消费上一帧） */
void de_lcd_apply_pending_host_blit(void);
#if MORAL_EMU_DEDICATED_THREAD
/** 模拟线程：在 uc_emu_start 返回后执行 Guest 显存 uc_mem_read，避免与 CPU 写屏并发撕裂 */
void de_emulator_service_guest_fb_pull(void);
#endif


typedef struct _lcdLayer
{
    u32 enable;
    u32 buffer_ptr;
    u32 width;
    u32 pitch;
    u32 height;
    u8 alpha;
    u8 alpha_enable;
    u8 src_key_enable;
    u8 rotate;
    u32 transparent_color;
    u32 offset_x;
    u32 offset_y;
    u32 memory_offx;
    u32 memory_offy;
    u16 *lcdBuffer;
    u16 pixel_byte_count;
} lcdLayer;

typedef struct _lcdRegionOfInterest
{
    u32 roi_sx;
    u32 roi_sy;
    u32 roi_width;
    u32 roi_height;
    u32 pitch;
    u32 background;
    u16 *lcdBuffer;
    u32 data_addr_type;
} lcdRegionOfInterest;

uint64_t render_time;
u32 lcdUpdateFlag = 0;
bool lcdIrqFlag = false;
bool needUpdateLCD;
bool LCD_Initialized = false;

// 最多四层
lcdLayer lcdLayerList[4];
lcdRegionOfInterest lcdRegionOfInterest_st;

#endif /* LCD_H */