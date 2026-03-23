#ifndef TOUCHSCREEN_H
#define TOUCHSCREEN_H

#include "main.h"
#include "vmEvent.h"

extern u32 isTouchDown;
extern u32 touchX;
extern u32 touchY;
extern u32 isInitTouch;


#define TS_CMD_ADDR_Y 0x0010
#define TS_CMD_ADDR_YN 0X0020
#define TS_CMD_ADDR_Z1 0x0030 // 用于检测压力
#define TS_CMD_ADDR_Z2 0x0040 // 用于检测压力
#define TS_CMD_ADDR_X 0x0050
#define TS_CMD_ADDR_XN 0X0060

#define AUX_BASE 0x82050000
#define AUX_UnkReg (AUX_BASE + 0x10)
#define AUX_TS_CMD 0x82050054
#define AUX_TS_CON 0x82050058
#define AUX_TS_DATA1 0x8205005C

/** 把当前 touchX/Y 写入 MTK 面板寄存器（0x3400C1xx），供固件轮询；鼠标事件里也会调用 */
void mtk_touch_regs_sync(void);
/** MEM_READ 钩子：读触摸相关寄存器前刷新，避免仅依赖中断时无坐标 */
void mtk_touch_hook_mem_read(uint64_t address);
/** 每次 pen-DOWN 时重置固件触摸状态（PressCount / 邮箱等），防止触摸状态机卡死 */
void moral_touch_on_pen_down(void);

#endif /* TOUCHSCREEN_H */
