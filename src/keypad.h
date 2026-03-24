#pragma once

#include "main.h"
#include "vmEvent.h"

#define KPD_STATUS_REG    0x34000814
#define KPD_DATA_REG      0x34000804
#define KPD_CLEAR_REG     0x3400080C
#define KPD_IRQ_LINE      12

typedef struct {
    u8 row;
    u8 col;
} KeyScanCode;

/*
 * Moral i9 触屏手机只有 5 个物理按键，映射表从固件运行时 dump：
 *   (0,0)=vkey40  (0,1)=vkey44
 *   (4,0)=vkey39  (4,1)=vkey38  (4,2)=vkey45
 */
static const KeyScanCode mstarKeyMap[] = {
    [0]  = {0, 0}, // KEY_0 (no numpad on touchscreen phone)
    [1]  = {0, 0}, // KEY_1
    [2]  = {0, 0}, // KEY_2
    [3]  = {0, 0}, // KEY_3
    [4]  = {0, 0}, // KEY_4
    [5]  = {0, 0}, // KEY_5
    [6]  = {0, 0}, // KEY_6
    [7]  = {0, 0}, // KEY_7
    [8]  = {0, 0}, // KEY_8
    [9]  = {0, 0}, // KEY_9
    [10] = {0, 0}, // KEY_STAR
    [11] = {0, 0}, // KEY_HASH
    [12] = {0, 0}, // unused
    [13] = {0, 0}, // unused
    [14] = {4, 1}, // KEY_UP → vkey38 (volume up / side up)
    [15] = {4, 0}, // KEY_DOWN → vkey39 (volume down / side down)
    [16] = {0, 0}, // KEY_LEFT (unused)
    [17] = {0, 0}, // KEY_RIGHT (unused)
    [18] = {0, 0}, // KEY_OK (unused)
    [19] = {0, 0}, // unused
    [20] = {0, 1}, // KEY_SKLEFT → vkey44 (function key)
    [21] = {4, 2}, // KEY_SKRIGHT → vkey45 (back)
    [22] = {0, 0}, // KEY_SEND (unused on touchscreen)
    [23] = {0xFF, 0xFF}, // KEY_END → 走 VM_EVENT_POWER_KEY 专用路径，不走矩阵
};
#define MSTAR_KEY_MAP_SIZE (sizeof(mstarKeyMap) / sizeof(mstarKeyMap[0]))

void handleKeyPadVmEvent(vm_event *vmEvent, uint64_t address);