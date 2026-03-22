#include "touchscreen.h"

u32 isTouchDown;
u32 touchX;
u32 touchY;
u32 isInitTouch;

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

/*
 * IDA MCP：段名 XRAM @0x00D00000，_gtTouchScreenSusbribeData 线性地址 0xD09198。
 */
static const u32 MMI_TOUCH_MAIL_BASES[] = {
    0xD09198u,
};

/*
 * 一次性初始化补丁：校准参数 + 固件环境安全修复。
 * 不再修改 PressCount、邮箱等运行时状态（固件自行管理）。
 */
void moral_touch_init_patch(void)
{
    u8 one = 1;
    u32 zero32 = 0;

    if (MTK == NULL)
        return;

    moral_touch_calibration_patch();

    /*
     * bLCDisOn @ 0xD007F8: DispUpdateScreenMdl 通过 DrvLcdCheckPowerStatus
     * 检查此变量，为 0 则跳过 DrvLcdUpdate，导致触摸后 DE trigger 不触发。
     */
    uc_mem_write(MTK, 0xD007F8u, &one, 1);

    for (unsigned bi = 0; bi < sizeof MMI_TOUCH_MAIL_BASES / sizeof MMI_TOUCH_MAIL_BASES[0]; bi++)
    {
        u32 base = MMI_TOUCH_MAIL_BASES[bi];
        u8 ts_mode = 1;
        u32 poll_ok = 60u;
        u32 poll;

        /*
         * _gnTsPrMode @ base - 0x42：DoMainJob 需 LCD 开或 gnTsPrMode==1 才走触摸 ADC。
         * 模拟器中 LCD 状态可能不准，强制设为 1。
         */
        uc_mem_write(MTK, base - 0x42u, &ts_mode, 1);

        /*
         * _gnPollingTime @ base - 4：为 0 时固件除法崩溃；过大时触摸不灵敏。
         * 仅在无效时设安全默认值。
         */
        if (uc_mem_read(MTK, base - 4u, &poll, 4) == UC_ERR_OK && (poll == 0u || poll > 5000u))
            uc_mem_write(MTK, base - 4u, &poll_ok, 4);
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
