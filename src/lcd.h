#ifndef LCD_H
#define LCD_H

#include "main.h"
#include "vmEvent.h"

#define LCD_BASE 0x90000000

/** 从模拟器 DE/LCD 缓冲区读取整屏并写入 SDL（供 lcdTaskMain 定时调用） */
void de_emulator_periodic_refresh(void);

void handleLcdReg(uint64_t address, u32 data, uint64_t value);
void compositeLayers(void);

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

clock_t render_time;
u32 lcdUpdateFlag = 0;
bool lcdIrqFlag = false;
bool needUpdateLCD;
bool LCD_Initialized = false;

// 最多四层
lcdLayer lcdLayerList[4];
lcdRegionOfInterest lcdRegionOfInterest_st;

#endif /* LCD_H */