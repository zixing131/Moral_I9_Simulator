#include "keypad.h"

/*
 * IDA DrvKeypadPowerOnOffKey(0x14ada)：电源/挂机键走独立路径，
 * 直接写 bIsPowerPressed@0xD00005 并触发键盘 IRQ。
 * 固件在 DrvKeypadISR 里检测到 bIsPowerPressed 后调用 DrvKeypadPowerOnOffKey 构造 mailbox 消息。
 */
#define GUEST_bIsPowerPressed 0xD00005u

void SimulateMstarPressKey(u8 key, u8 is_press)
{
    if (key >= MSTAR_KEY_MAP_SIZE)
        return;

    u8 row = mstarKeyMap[key].row;
    u8 col = mstarKeyMap[key].col;

    if (row == 0 && col == 0 && key != 4)
        return;

    u32 status;
    if (is_press)
    {
        u8 rowByte = 0xFF & ~(1 << row);
        u8 colByte = 0xFF & ~(1 << col);
        status = (colByte << 8) | rowByte;
    }
    else
    {
        status = 0xFFFF;
    }
    uc_mem_write(MTK, KPD_STATUS_REG, &status, 4);
}

/*
 * IDA DrvKeypadPowerOnOffKey(0x14ada)：电源键由独立 PMU 中断调用，不走键盘矩阵 ISR。
 * 直接通过 Unicorn 调用固件函数：保存/恢复寄存器，设 R0=press，PC 跳到函数入口执行。
 */
static void SimulatePowerKey(u8 is_press, uint64_t address)
{
    u32 saved_r[16];
    u32 saved_cpsr;
    for (int i = 0; i < 16; i++)
        uc_reg_read(MTK, UC_ARM_REG_R0 + i, &saved_r[i]);
    uc_reg_read(MTK, UC_ARM_REG_CPSR, &saved_cpsr);

    u32 arg = is_press ? 1 : 0;
    uc_reg_write(MTK, UC_ARM_REG_R0, &arg);
    /* LR 设为一个安全地址让函数返回后停下；用 address（当前 PC）即可 */
    u32 lr_val = (u32)address | 1u; /* Thumb return */
    uc_reg_write(MTK, UC_ARM_REG_LR, &lr_val);

    /* 调用 DrvKeypadPowerOnOffKey @ 0x14ADA（Thumb 入口 +1） */
    uc_emu_start(MTK, 0x14ADBu, (u32)address, 0, 200000);

    for (int i = 0; i < 16; i++)
        uc_reg_write(MTK, UC_ARM_REG_R0 + i, &saved_r[i]);
    uc_reg_write(MTK, UC_ARM_REG_CPSR, &saved_cpsr);
}

inline void handleKeyPadVmEvent(vm_event *vmEvent, uint64_t address)
{
    if (vmEvent->event == VM_EVENT_KEYBOARD)
    {
        u8 key = (u8)vmEvent->r0;
        u8 press = (u8)vmEvent->r1;
        SimulateMstarPressKey(key, press);
        if (!StartInterrupt(KPD_IRQ_LINE, address))
            EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);
    }
    else if (vmEvent->event == VM_EVENT_POWER_KEY)
    {
        u8 press = (u8)vmEvent->r0;
        SimulatePowerKey(press, address);
    }
}
