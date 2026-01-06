#pragma once

#include "main.h"
#include "vmEvent.h"

/**
 * 在powerKeyPress()中断查找此表地址
 */

u8 keypaddef[77];

#define KEYBOARD_REG_BASE 0x81070000
#define KEYPAD_STATUS (KEYBOARD_REG_BASE + 0x00)
#define KEYPAD_SCAN_OUT_MEM1 (KEYBOARD_REG_BASE + 0x04)
#define KEYPAD_SCAN_OUT_MEM2 (KEYBOARD_REG_BASE + 0x08)
#define KEYPAD_SCAN_OUT_MEM3 (KEYBOARD_REG_BASE + 0x0C)
#define KEYPAD_SCAN_OUT_MEM4 (KEYBOARD_REG_BASE + 0x10)
#define KEYPAD_DOUBLE (KEYBOARD_REG_BASE + 0x18)

void handleKeyPadVmEvent(vm_event *vmEvent, uint64_t address);