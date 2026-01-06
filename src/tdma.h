#include "main.h"

typedef struct VM_TDMA_STATE
{
    u32 wrap_cnt;
    u32 tq_cnt;
    u32 evt_val;
    u32 has_irq;
} VM_TDMA_STATE;

VM_TDMA_STATE TDMA_state;

#define TDMA_BASE 0x82000000
#define TDMA_TQCNT (TDMA_BASE + 0x0)
#define TDMA_WRAP (TDMA_BASE + 0x4)
#define TDMA_WRAPIMD (TDMA_BASE + 0x8)
#define TDMA_EVTVAL (TDMA_BASE + 0xc)
#define TDMA_DTIRQ (TDMA_BASE + 0x10)

// 接收中断
#define TDMA_RX_CON (TDMA_BASE + 0x184)

#define L1SM_CONTROL (TDMA_BASE + 0x218)
#define L1SM_STAT (TDMA_BASE + 0x21C)
#define L1SM_DURATION (TDMA_BASE + 0x220)
#define L1SM_MSB_MESSURE_RESULT (TDMA_BASE + 0x224)
#define L1SM_LSB_MESSURE_RESULT (TDMA_BASE + 0x228)
#define L1SM_CONF (TDMA_BASE + 0x22C)


u32 last_timer_interrupt_time;
