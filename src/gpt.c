#include "gpt.h"
#include "vmEvent.h"

static clock_t gpt_ticks_to_ms(uint64_t ticks, u32 prescaler)
{
    u32 ps = prescaler & 7u;
    uint64_t freq_khz = 16000u >> ps; /* 16MHz / 2^ps, in kHz */
    if (freq_khz == 0) freq_khz = 1;
    clock_t ms = (clock_t)(ticks / freq_khz);
    if (ms < 1) ms = 1;
    return ms;
}

void handleGptReg(uint64_t address, u32 data, uint64_t value)
{
    switch (address)
    {
    case GPT1_CONTROL:
        if (data == 1)
        {
            gpt1Enable = ((value & 0x8000) == 0x8000);
            gpt1Circle = ((value & 0x4000) == 0x4000);
            printf("[GPT-CFG] GPT1_CONTROL val=0x%llx en=%d circle=%d\n",
                   (unsigned long long)value, gpt1Enable, gpt1Circle);
        }
        break;
    case GPT2_CONTROL:
        if (data == 1)
        {
            gpt2Enable = ((value & 0x8000) == 0x8000);
            gpt2Circle = ((value & 0x4000) == 0x4000);
            printf("[GPT-CFG] GPT2_CONTROL val=0x%llx en=%d circle=%d\n",
                   (unsigned long long)value, gpt2Enable, gpt2Circle);
        }
        break;
    case GPT1_TOUT_INTERVAL:
        if (data == 1)
        {
            gpt1Tout = value;
            if (value > 0)
            {
                clock_t ms = gpt_ticks_to_ms(value, gpt1Prescaler);
                last_gpt1_interrupt_time = currentTime + ms;
                printf("[GPT-CFG] GPT1_TOUT raw=%llu prescaler=%u -> %ldms\n",
                       (unsigned long long)value, gpt1Prescaler, (long)ms);
            }
        }
        break;
    case GPT2_TOUT_INTERVAL:
        if (data == 1)
        {
            gpt2Tout = value;
            if (value > 0)
            {
                clock_t ms = gpt_ticks_to_ms(value, gpt2Prescaler);
                last_gpt2_interrupt_time = currentTime + ms;
                printf("[GPT-CFG] GPT2_TOUT raw=%llu prescaler=%u -> %ldms\n",
                       (unsigned long long)value, gpt2Prescaler, (long)ms);
            }
        }
        break;
    case GPT1_PRESCALER:
        if (data == 1)
        {
            gpt1Prescaler = (u32)value & 7u;
            printf("[GPT-CFG] GPT1_PRESCALER=%u\n", gpt1Prescaler);
        }
        break;
    case GPT2_PRESCALER:
        if (data == 1)
        {
            gpt2Prescaler = (u32)value & 7u;
            printf("[GPT-CFG] GPT2_PRESCALER=%u\n", gpt2Prescaler);
        }
        break;
    }
}

void GptTaskMain(void)
{
    if (gpt1Enable && last_gpt1_interrupt_time > 0 && currentTime >= last_gpt1_interrupt_time)
    {
        if (gpt1Circle)
        {
            clock_t ms = gpt_ticks_to_ms(gpt1Tout, gpt1Prescaler);
            last_gpt1_interrupt_time = currentTime + ms;
        }
        else
            last_gpt1_interrupt_time = 0;
        EnqueueVMEvent(VM_EVENT_GPT_IRQ, 1, 0);
    }

    if (gpt2Enable && last_gpt2_interrupt_time > 0 && currentTime >= last_gpt2_interrupt_time)
    {
        if (gpt2Circle)
        {
            clock_t ms = gpt_ticks_to_ms(gpt2Tout, gpt2Prescaler);
            last_gpt2_interrupt_time = currentTime + ms;
        }
        else
            last_gpt2_interrupt_time = 0;
        EnqueueVMEvent(VM_EVENT_GPT_IRQ, 2, 0);
    }
}