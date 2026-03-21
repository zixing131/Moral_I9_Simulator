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

static const KeyScanCode mstarKeyMap[] = {
    [0]  = {2, 7}, // KEY_0
    [1]  = {1, 5}, // KEY_1
    [2]  = {1, 6}, // KEY_2
    [3]  = {1, 7}, // KEY_3
    [4]  = {2, 0}, // KEY_4
    [5]  = {2, 1}, // KEY_5
    [6]  = {2, 2}, // KEY_6
    [7]  = {2, 3}, // KEY_7
    [8]  = {2, 4}, // KEY_8
    [9]  = {2, 5}, // KEY_9
    [10] = {2, 6}, // KEY_STAR
    [11] = {3, 0}, // KEY_HASH
    [12] = {0, 0}, // unused
    [13] = {0, 0}, // unused
    [14] = {4, 6}, // KEY_UP (SIDE_UP)
    [15] = {4, 7}, // KEY_DOWN (SIDE_DN)
    [16] = {0, 0}, // KEY_LEFT (unused)
    [17] = {0, 0}, // KEY_RIGHT (unused)
    [18] = {0, 0}, // KEY_OK (unused)
    [19] = {0, 0}, // unused
    [20] = {5, 1}, // KEY_SKLEFT
    [21] = {5, 1}, // KEY_SKRIGHT
    [22] = {1, 1}, // KEY_SEND
    [23] = {0, 0}, // KEY_END (unused)
};
#define MSTAR_KEY_MAP_SIZE (sizeof(mstarKeyMap) / sizeof(mstarKeyMap[0]))

void handleKeyPadVmEvent(vm_event *vmEvent, uint64_t address);