#include "keypad.h"

void SimulatePressKey(u8 key, u8 is_press)
{
    u8 kv;
    bool found = false;
    for (u8 i = 0; i < 72; i++)
    {
        if (keypaddef[i] == key)
        {
            found = true;
            kv = i;
            break;
        }
    }
    if (found)
    {
        // kv是对应的寄存器第几位
        changeTmp = 1;                                   // 状态改变
        uc_mem_write(MTK, KEYPAD_STATUS, &changeTmp, 4); // 有按键按下
        changeTmp = (kv >= 0 && kv < 16) ? (is_press << kv) : 0;
        changeTmp = 0xffff & (~changeTmp);
        uc_mem_write(MTK, KEYPAD_SCAN_OUT_MEM1, &changeTmp, 2);
        changeTmp = (kv >= 16 && kv < 32) ? (is_press << (kv - 16)) : 0;
        changeTmp = 0xffff & (~changeTmp);
        uc_mem_write(MTK, KEYPAD_SCAN_OUT_MEM2, &changeTmp, 2);
        changeTmp = (kv >= 32 && kv < 48) ? (is_press << (kv - 32)) : 0;
        changeTmp = 0xffff & (~changeTmp);
        uc_mem_write(MTK, KEYPAD_SCAN_OUT_MEM3, &changeTmp, 2);
        changeTmp = (kv >= 48 && kv < 64) ? (is_press << (kv - 48)) : 0;
        changeTmp = 0xffff & (~changeTmp);
        uc_mem_write(MTK, KEYPAD_SCAN_OUT_MEM4, &changeTmp, 2);
    }
}



inline void handleKeyPadVmEvent(vm_event *vmEvent, uint64_t address)
{

    if (vmEvent->event == VM_EVENT_KEYBOARD)
    {
        // 按键中断
        if (StartInterrupt(DEV_IRQ_CHANNEL_KEYBOARD, address))
            SimulatePressKey(vmEvent->r0, vmEvent->r1);
        else // 如果处理失败，重新入队
            EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);
    }
}

