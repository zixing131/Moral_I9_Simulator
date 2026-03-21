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
 * IDA MCP：段名 XRAM @0x00D00000，_gtTouchScreenSusbribeData 线性地址 0xD09198（即 0x00D09198）。
 * 不是 0x0D000000+0x9198（那是 218MB 带，与 scatter 无关）。
 */
static const u32 MMI_TOUCH_MAIL_BASES[] = {
    0xD09198u,
};

/*
 * 校准地址（IDA XRAM 段）：
 * _gtCalibrationInfo @ 0xD0F3D8  —  X offset
 *       dword_D0F3DC              —  X scale
 *       dword_D0F3E0              —  Y offset
 *       dword_D0F3E4              —  Y scale
 *
 * 固件转换公式：
 *   screenX = (adcX - X_offset) * X_scale / 4096
 *   screenY = (adcY - Y_offset) * Y_scale / 4096
 *
 * 模拟器 ADC 映射：adcX = touchX*1023/240,  adcY = touchY*1023/400
 * 令 offset=0，scale = 4096 * screenDim / 1023
 *   X_scale = 4096*240/1023 ≈ 961
 *   Y_scale = 4096*400/1023 ≈ 1602
 */
#define CALIB_X_OFFSET_ADDR 0xD0F3D8u
#define CALIB_X_SCALE_ADDR  0xD0F3DCu
#define CALIB_Y_OFFSET_ADDR 0xD0F3E0u
#define CALIB_Y_SCALE_ADDR  0xD0F3E4u

static void moral_touch_calibration_patch(void)
{
    u32 v;
    v = 0;    uc_mem_write(MTK, CALIB_X_OFFSET_ADDR, &v, 4);
    v = 961;  uc_mem_write(MTK, CALIB_X_SCALE_ADDR,  &v, 4);
    v = 0;    uc_mem_write(MTK, CALIB_Y_OFFSET_ADDR, &v, 4);
    v = 1602; uc_mem_write(MTK, CALIB_Y_SCALE_ADDR,  &v, 4);
}

static void moral_touch_mmi_ram_patch(void)
{
    u8 one = 1;
    u8 ts_mode = 1;
    u32 zero32 = 0;
    u32 poll_ok = 60u;
    unsigned bi;

    if (MTK == NULL)
        return;

    moral_touch_calibration_patch();

    /*
     * bLCDisOn @ 0xD007F8: DispUpdateScreenMdl 通过 DrvLcdCheckPowerStatus
     * 检查此变量，为 0 则跳过 DrvLcdUpdate，导致触摸后 DE trigger 不触发。
     */
    uc_mem_write(MTK, 0xD007F8u, &one, 1);

    for (bi = 0; bi < sizeof MMI_TOUCH_MAIL_BASES / sizeof MMI_TOUCH_MAIL_BASES[0]; bi++)
    {
        u32 base = MMI_TOUCH_MAIL_BASES[bi];
        u16 mbox;
        u32 poll;
        /* +2 byte_D0919A，+4 gReEnableTouchScreenFlag（相对邮箱半字） */
        if (uc_mem_write(MTK, base + 2u, &one, 1) != UC_ERR_OK)
            continue;
        uc_mem_write(MTK, base + 4u, &zero32, 1);
        if (uc_mem_read(MTK, base, &mbox, 2) == UC_ERR_OK && mbox == 255u)
        {
            u16 z = 0;
            uc_mem_write(MTK, base, &z, 2);
        }
        /*
         * _gnTsPrMode @ 0xd09156 = base - 0x42：DoMainJob 需 LCD 开或 gnTsPrMode==1 才走触摸 ADC
         * _gnTouchScreenPressCount @ 0xd09164 = base - 0x34：过大时 LABEL_27 直接 return，永不 MsSend
         * _gnPollingTime @ 0xd09194 = base - 4：为 0 或未初始化除法会崩；过大时 (T+599)/T==1 一次上报后即被屏蔽
         */
        uc_mem_write(MTK, base - 0x42u, &ts_mode, 1);
        uc_mem_write(MTK, base - 0x34u, &zero32, 4);
        if (uc_mem_read(MTK, base - 4u, &poll, 4) == UC_ERR_OK && (poll == 0u || poll > 5000u))
            uc_mem_write(MTK, base - 4u, &poll_ok, 4);
        /* 按压时清释放计数，避免卡在释放状态机 */
        if (isTouchDown)
            uc_mem_write(MTK, base - 0x0cu, &zero32, 4);
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

    static u8 patch_done = 0;
    if (!patch_done)
    {
        patch_done = 1;
        moral_touch_mmi_ram_patch();
    }
}

void mtk_touch_hook_mem_read(uint64_t address)
{
    (void)address;
    mtk_touch_regs_sync();
}

void moral_touch_on_pen_down(void)
{
    u32 zero32 = 0;
    u8 one = 1;
    if (MTK == NULL)
        return;

    for (unsigned bi = 0; bi < sizeof MMI_TOUCH_MAIL_BASES / sizeof MMI_TOUCH_MAIL_BASES[0]; bi++)
    {
        u32 base = MMI_TOUCH_MAIL_BASES[bi];
        u16 mbox;

        /*
         * _gbTSState @ base-0x28 (0xD09170): MdlTouchScreenPenDetect 仅在 state==0 时
         * 接受新的 pen-detect。强制重置确保新的按下事件不被拒绝。
         */
        uc_mem_write(MTK, base - 0x28u, &zero32, 4);

        /* _gncurrentAdcJob @ base-0x48 (0xD09150): 重置 ADC 双采样状态 */
        uc_mem_write(MTK, base - 0x48u, &zero32, 1);

        /* _gnTouchScreenPressCount @ base-0x34: 过大时固件跳过 MsSend */
        uc_mem_write(MTK, base - 0x34u, &zero32, 4);

        /* _gnTouchADCRepeatCounter @ base-0x2C (0xD0916C): 重置 ADC 重试计数 */
        uc_mem_write(MTK, base - 0x2Cu, &zero32, 4);

        /* _gnCounterFailureTimes @ base-0x08 (0xD09190): 重置失败计数 */
        uc_mem_write(MTK, base - 0x08u, &zero32, 4);

        /* 邮箱饱和时重置，避免 MsSend 发送失败 */
        if (uc_mem_read(MTK, base, &mbox, 2) == UC_ERR_OK && mbox >= 200u)
        {
            u16 z = 0;
            uc_mem_write(MTK, base, &z, 2);
        }

        /* 确保触摸使能标志有效 */
        uc_mem_write(MTK, base + 2u, &one, 1);

        /* 清释放计数，防止卡在释放状态机 */
        uc_mem_write(MTK, base - 0x0cu, &zero32, 4);
    }
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
