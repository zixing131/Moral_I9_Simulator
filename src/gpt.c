#include "gpt.h"
#include "vmEvent.h"

void handleGptReg(uint64_t address, u32 data, uint64_t value)
{
    switch (address)
    {
    case GPT1_CONTROL:
        if (data == 1)
        {
            gpt1Enable = ((value & 0x8000) == 0x8000);
            gpt1Circle = ((value & 0x4000) == 0x4000);
        }
        break;
    case GPT2_CONTROL:
        if (data == 1)
        {
            gpt2Enable = ((value & 0x8000) == 0x8000);
            gpt2Circle = ((value & 0x4000) == 0x4000);
        }
        break;
    case GPT1_TOUT_INTERVAL:
        if (data == 1)
        {
            gpt1Tout = value;
            if (value > 0)
                last_gpt1_interrupt_time = currentTime + value;
        }
        break;
    case GPT2_TOUT_INTERVAL:
        if (data == 1)
        {
            gpt2Tout = value;
            if (value > 0)
                last_gpt2_interrupt_time = currentTime + value;
        }
        break;
    }
}

void GptTaskMain()
{
    if (gpt1Enable && last_gpt1_interrupt_time > 0 && currentTime >= last_gpt1_interrupt_time)
    {
        if (gpt1Circle)
            last_gpt1_interrupt_time = currentTime + gpt1Tout;
        else
            last_gpt1_interrupt_time = 0;
        EnqueueVMEvent(VM_EVENT_GPT_IRQ, 1, 0);
    }

    if (gpt2Enable && last_gpt2_interrupt_time > 0 && currentTime >= last_gpt2_interrupt_time)
    {
        if (gpt2Circle)
            last_gpt2_interrupt_time = currentTime + gpt2Tout;
        else
            last_gpt2_interrupt_time = 0;
        EnqueueVMEvent(VM_EVENT_GPT_IRQ, 2, 0);
    }
}