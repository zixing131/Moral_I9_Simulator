#include "tdma.h"

u8 TDMATaskMain()
{
    u32 tmp;
    {
        uc_reg_read(MTK, UC_ARM_REG_CPSR, &tmp);
        if (TDMA_state.tq_cnt == 0)
            TDMA_state.tq_cnt = 2;
        else if (TDMA_state.tq_cnt < TDMA_state.evt_val)
        {
            TDMA_state.tq_cnt += 2000;
            TDMA_state.tq_cnt = min(TDMA_state.tq_cnt, TDMA_state.evt_val);
        }
        else if (TDMA_state.tq_cnt >= TDMA_state.evt_val && TDMA_state.tq_cnt < TDMA_state.wrap_cnt)
        {
            TDMA_state.tq_cnt += 200;
            TDMA_state.tq_cnt = min(TDMA_state.tq_cnt, TDMA_state.wrap_cnt);
        }
        else if (TDMA_state.tq_cnt == TDMA_state.wrap_cnt)
            TDMA_state.tq_cnt = 0;

        *(int *)(RAM82000000_POOL) = TDMA_state.tq_cnt;
        if (TDMA_state.tq_cnt == TDMA_state.evt_val)
        {
            EnqueueVMEvent(VM_EVENT_Timer_IRQ, DEV_IRQ_CHANNEL_CTIRQ1, 0);
        }
    }
}

void handleTdmaReg(uint64_t address, u32 data, uint64_t value)
{
    u32 tmp;
    switch (address)
    {
    case TDMA_EVTVAL:
        if (data == 1)
        {
            TDMA_state.evt_val = value + 2;
            TDMA_state.tq_cnt = 0;
            // printf("set TDMA EVTVAL %x\n", value);
            // printf("at %x\n", lastAddress);
        }
        break;
    case TDMA_WRAP:
        if (data == 1)
        {
            TDMA_state.wrap_cnt = value;
            // printf("set TDMA Wrap %x\n", value);
        }
        break;
    // case TDMA_TQCNT:
    //     if (data == 0)
    //     {
    //         *(int *)(RAM82000000_POOL) = TDMA_state.tq_cnt;
    //     }
    //     break;
    default:
        // if (data == 1 && address <= (TDMA_BASE + 0xf000) && address >= TDMA_BASE){
        //     printf("write tdma %x ", address);
        //     printf("%x \n",value);

        // }
        break;
    }
}