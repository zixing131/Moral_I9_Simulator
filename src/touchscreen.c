#include "touchscreen.h"

u32 isTouchDown;
u32 touchX;
u32 touchY;
u32 isInitTouch;

/* 与 vmEvent.c 中触摸中断写入保持一致，便于固件轮询 0x3400C1xx */
#define MTK_TP_REG_STATUS 0x3400C1BCu
#define MTK_TP_REG_UNK    0x3400C1C4u
#define MTK_TP_REG_X      0x3400C1C0u
#define MTK_TP_REG_Y      0x3400C1C8u

/*
 * MMI 内部 RAM 触摸全局量（IDA：_gtTouchScreenSusbribeData 0xd09198、byte_D0919A、gReEnableTouchScreenFlag）
 * 链接基址按 0x0D000000 + 偏移，与 uc_mem_map_ptr(0x0d000000, ...) 一致。
 * - byte_D0919A：非 0 才走 MdlTouchScreenStatusReport
 * - gReEnableTouchScreenFlag：为 0 时 _MdlTouchscreenRepeatADCProcess 走失败/诊断分支（日志里 0x18000918/0x8000918）
 */
#define MMI_TOUCH_GT_MAIL_U16   0x0D009198u
#define MMI_TOUCH_REPORT_EN_U8  0x0D00919Au
#define MMI_TOUCH_REENABLE_U8   0x0D00919Cu

static void moral_touch_mmi_ram_patch(void)
{
    u8 one = 1;
    u16 mbox;

    if (MTK == NULL)
        return;
    uc_mem_write(MTK, MMI_TOUCH_REPORT_EN_U8, &one, 1);
    uc_mem_write(MTK, MMI_TOUCH_REENABLE_U8, &one, 1);
    if (uc_mem_read(MTK, MMI_TOUCH_GT_MAIL_U16, &mbox, 2) == UC_ERR_OK && mbox == 255u)
    {
        u16 z = 0;
        uc_mem_write(MTK, MMI_TOUCH_GT_MAIL_U16, &z, 2);
    }
}

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

    moral_touch_mmi_ram_patch();
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
