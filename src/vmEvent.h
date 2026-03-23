#pragma once
#include "main.h"
#include "vmDefine.h"

typedef enum
{
    VM_EVENT_NONE,
    VM_EVENT_KEYBOARD,
    VM_EVENT_Timer_IRQ,
    VM_EVENT_MSDC_IO_OK,
    VM_EVENT_MSDC_IO_CALLBACK,
    VM_EVENT_RTC_IRQ,
    VM_EVENT_GPT_IRQ,
    VM_EVENT_SIM_IRQ,
    VM_EVENT_SIM_T0_TX_END,
    VM_EVENT_SIM_T0_RX_END,
    VM_EVENT_DMA_IRQ,
    VM_EVENT_MSDC_IRQ,
    VM_EVENT_UART_IRQ,
    VM_EVENT_LCD_IRQ,
    VM_EVENT_TOUCH_SCREEN_IRQ,
    VM_EVENT_L1SM_IRQ,
    VM_EVENT_L1D_DSP_IRQ,
    VM_EVENT_AUDIO_DSP_IRQ,
    VM_EVENT_I2C_IRQ,
    VM_EVENT_EXIT

} VM_EVENT;

typedef struct vm_event_
{
    u32 event;
    u32 r0;
    u32 r1;
} vm_event;

static u8 timer_irq_pending = 0;
void InitVmEvent();
void handleVmEvent_EMU(uint64_t address);
int EnqueueVMEvent(u32 event, u32 r0, u32 r1);
vm_event *DequeueVMEvent();
/** 鼠标按下/拖动时立即置位 ADC 轮询（不依赖 VM 队列 1000 block 延迟与 IRQ31 是否成功） */
void moral_vm_touch_adc_request(u32 x, u32 y);