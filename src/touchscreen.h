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
#define TS_CMD_ADDR_Z1 0x0030
#define TS_CMD_ADDR_Z2 0x0040
#define TS_CMD_ADDR_X 0x0050
#define TS_CMD_ADDR_XN 0X0060

#define AUX_BASE 0x82050000
#define AUX_UnkReg (AUX_BASE + 0x10)
#define AUX_TS_CMD 0x82050054
#define AUX_TS_CON 0x82050058
#define AUX_TS_DATA1 0x8205005C

/** 一次性初始化补丁：校准参数 + 固件环境安全修复 */
void moral_touch_init_patch(void);

#endif /* TOUCHSCREEN_H */
