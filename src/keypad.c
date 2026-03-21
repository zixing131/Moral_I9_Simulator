#include "keypad.h"

void SimulatePressKey(u8 key, u8 is_press)
{
    if (key >= MSTAR_KEY_MAP_SIZE)
        return;

    u8 row = mstarKeyMap[key].row;
    u8 col = mstarKeyMap[key].col;

    if (row == 0 && col == 0 && key != 4 && key != 23)
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
        u8 key = (u8)vmEvent->r0;
        u8 press = (u8)vmEvent->r1;
        static u32 kpd_log = 0;
        if (kpd_log < 20)
        {
            kpd_log++;
            u8 r = (key < MSTAR_KEY_MAP_SIZE) ? mstarKeyMap[key].row : 0xFF;
            u8 c = (key < MSTAR_KEY_MAP_SIZE) ? mstarKeyMap[key].col : 0xFF;
            printf("[KPD] key=%u press=%u row=%u col=%u\n", key, press, r, c);
        }
        if (kpd_log == 1)
        {
            u8 mbox_val = 0xFF;
            uc_mem_read(MTK, 0xD00000u, &mbox_val, 1);
            printf("[KPD] gKeypadMailbox=0x%02x (at 0xD00000)\n", mbox_val);

            u8 tbl_idx = 0;
            uc_mem_read(MTK, 0xD00006u, &tbl_idx, 1);
            u32 base = 0x8355e0u + 94u * tbl_idx + 30u;
            u8 keymap[64];
            if (uc_mem_read(MTK, base, keymap, 64) == UC_ERR_OK)
            {
                printf("[KPD] KEY TABLE (tblIdx=%u base=0x%x):\n", tbl_idx, base);
                for (int r = 0; r < 8; r++)
                {
                    printf("[KPD]  row%d:", r);
                    for (int c = 0; c < 8; c++)
                        printf(" %02x", keymap[r * 8 + c]);
                    printf("\n");
                }
            }
        }
        SimulatePressKey(key, press);
        if (!StartInterrupt(KPD_IRQ_LINE, address))
            EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);
        else if (kpd_log <= 20)
            printf("[KPD] IRQ12 fired OK\n");
    }
}

