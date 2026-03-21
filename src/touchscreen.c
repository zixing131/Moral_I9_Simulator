#include "touchscreen.h"

/* 与 vmEvent.c 中触摸中断写入保持一致，便于固件轮询 0x3400C1xx */
#define MTK_TP_REG_STATUS 0x3400C1BCu
#define MTK_TP_REG_UNK    0x3400C1C4u
#define MTK_TP_REG_X      0x3400C1C0u
#define MTK_TP_REG_Y      0x3400C1C8u

void mtk_touch_regs_sync(void)
{
    u32 tmp;
    if (MTK == NULL)
        return;
    u32 sx = touchX;
    u32 sy = touchY;
    if (sx >= 240u)
        sx = 239u;
    if (sy >= 400u)
        sy = 399u;

    tmp = sx * 1023u / 240u;
    uc_mem_write(MTK, MTK_TP_REG_X, &tmp, 4);
    tmp = sy * 1023u / 400u;
    uc_mem_write(MTK, MTK_TP_REG_Y, &tmp, 4);

    if (isTouchDown)
    {
        tmp = 3;
        uc_mem_write(MTK, MTK_TP_REG_STATUS, &tmp, 4);
        tmp = 1;
        uc_mem_write(MTK, MTK_TP_REG_UNK, &tmp, 4);
    }
    else
    {
        tmp = 0;
        uc_mem_write(MTK, MTK_TP_REG_STATUS, &tmp, 4);
        uc_mem_write(MTK, MTK_TP_REG_UNK, &tmp, 4);
    }
}

void mtk_touch_hook_mem_read(uint64_t address)
{
    (void)address;
    mtk_touch_regs_sync();
}

void InitTouchScreen()
{
    isTouchDown = 0;
    isInitTouch = 0;
    touchX = 0;
    touchY = 0;
}

void handleTouchScreenReg(uint64_t address, u32 data, uint64_t value)
{
    u32 tmp;
    switch (address)
    {
    case AUX_TS_CMD:
        if (data == 1)
        {
            if (value == TS_CMD_ADDR_X)
            {
                changeTmp1 = touchX * 1023 / 240;
                uc_mem_write(MTK, AUX_TS_DATA1, &changeTmp1, 4);
            }
            else if (value == TS_CMD_ADDR_Y)
            {
                changeTmp1 = touchY * 1023 / 400;
                uc_mem_write(MTK, AUX_TS_DATA1, &changeTmp1, 4);
            }
            else if (value == TS_CMD_ADDR_Z1)
            {
                changeTmp1 = isTouchDown == 1 ? 32 : 64;
                uc_mem_write(MTK, AUX_TS_DATA1, &changeTmp1, 4);
            }
            else if (value == TS_CMD_ADDR_Z2)
            {
                changeTmp1 = isTouchDown == 1 ? 64 : 32;
                uc_mem_write(MTK, AUX_TS_DATA1, &changeTmp1, 4);
            }
            else
            {
                changeTmp1 = 0x350;
                uc_mem_write(MTK, AUX_TS_DATA1, &changeTmp1, 4);
            }
        }
        break;
    case AUX_BASE:
        if (data == 0)
        {
            tmp = 0;
            uc_mem_write(MTK, AUX_BASE, &tmp, 4);
            tmp = 0x350;
            uc_mem_write(MTK, AUX_UnkReg, &tmp, 4);
        }
        break;
    case AUX_TS_CON:
        if (data == 0)
        {
            tmp = 0;
            uc_mem_write(MTK, (u32)address, &tmp, 4);
        }
        break;
    }
}
