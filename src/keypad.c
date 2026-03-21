#include "keypad.h"

void SimulatePressKey(u8 key, u8 is_press)
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

inline void handleKeyPadVmEvent(vm_event *vmEvent, uint64_t address)
{
    if (vmEvent->event == VM_EVENT_KEYBOARD)
    {
        SimulatePressKey(vmEvent->r0, vmEvent->r1);
        if (!StartInterrupt(KPD_IRQ_LINE, address))
            EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);
    }
}

