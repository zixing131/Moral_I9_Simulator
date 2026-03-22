#include "vmEvent.h"
#include "touchscreen.h"
#define MAX_VM_EVENT_COUNT 512
#define MAX_VM_EVENT_WAIT_COUNT 512

extern u32 aux_irq_en;
extern u32 aux_irq_sts;
extern volatile u8 pen_detect_pending;

u32 keyRowIdx = 0;
u32 keyColIdx = 0;
vm_event *firstEvent;
u32 vmIsLock;
u32 VmEventWaitCount;
vm_event VmEventHandleList[MAX_VM_EVENT_COUNT];
vm_event VmEventHandleWaitList[MAX_VM_EVENT_WAIT_COUNT];
vm_event currentEvent;
u32 VmEventCount = 0;
bool VmEventMutex;
vm_event *vmEvent;

void InitVmEvent()
{
    firstEvent = &VmEventHandleList[0];
    VmEventCount = 0;
    VmEventWaitCount = 0;
}

int EnqueueVMEvent(u32 event, u32 r0, u32 r1)
{
    u32 i;
    if (VmEventCount < MAX_VM_EVENT_COUNT)
    {
        if (vmIsLock == 0)
        {
            vmIsLock = 1;
            for (i = 0; i < VmEventWaitCount; i++)
            {
                if (VmEventCount >= MAX_VM_EVENT_COUNT)
                    break;
                VmEventHandleList[VmEventCount++] = VmEventHandleWaitList[i];
            }
            VmEventWaitCount = 0;
            vm_event *evt = &VmEventHandleList[VmEventCount++];
            evt->event = event;
            evt->r0 = r0;
            evt->r1 = r1;
            vmIsLock = 0;
        }
        else
        {
                if (VmEventWaitCount < MAX_VM_EVENT_WAIT_COUNT)
                {
                    vm_event *evt = &VmEventHandleWaitList[VmEventWaitCount++];
                    evt->event = event;
                    evt->r0 = r0;
                    evt->r1 = r1;
                }
            else
                printf("WARNING:Max VmEventWaitCount\n");
        }
        return 1;
    }
#ifdef sGDB_SERVER_SUPPORT
    else
    {
        printf("max vm event count\n");
        uc_emu_stop(MTK);
        isBreakPointHit = 1;
    }
#endif
    return 0;
}

inline vm_event *DequeueVMEvent()
{
    vm_event *evt;
    vm_event *ta;
    vm_event *tb;
    u32 i;
    if (VmEventCount > 0 && vmIsLock == 0)
    {
        vmIsLock = 1;
        ta = &VmEventHandleList[0];
        currentEvent.event = ta->event;
        currentEvent.r0 = ta->r0;
        currentEvent.r1 = ta->r1;
        evt = &currentEvent;
        --VmEventCount;
        for (i = 0; i < VmEventCount; i++)
        {
            ta = (&VmEventHandleList[i]);
            tb = ta + 1;
            ta->event = tb->event;
            ta->r0 = tb->r0;
            ta->r1 = tb->r1;
        }
        vmIsLock = 0;
    }
    else
        evt = 0;
    return evt;
}

uint64_t handleTick;

volatile u8 timer_event_pending = 0;
volatile u8 soft_timer_event_pending = 0;
volatile u8 irq13_chained = 0;

inline void handleVmEvent_EMU(uint64_t address)
{
    u32 tmp;

    /*
     * Cursor DE DMA delay: simulate real hardware latency using a
     * basic-block counter. Each call to handleVmEvent_EMU is one
     * basic block. After CURSOR_DE_DELAY_BLOCKS blocks, signal
     * DE completion and break the cursor-redraw storm.
     */
    if (cursor_de_pending)
    {
        cursor_de_block_count++;
        if (cursor_de_block_count >= CURSOR_DE_DELAY_BLOCKS)
        {
            cursor_de_pending = 0;
            tmp = 0xF00;
            uc_mem_write(MTK, 0x74003148u, &tmp, 4);
        }
    }

    /*
     * 定时器中断投递：由 MainUpdateTask 每 5ms 设置 timer_event_pending=1，
     * 此处在 IRQ 自然启用的窗口中尝试投递 IRQ 14。
     * 若 IRQ 被禁用则跳过，等待下一个 IRQ 启用的 basic block 再投递。
     */
    if (timer_event_pending)
    {
        u32 cpsr_t;
        uc_reg_read(MTK, UC_ARM_REG_CPSR, &cpsr_t);
        if (!isIRQ_Disable(cpsr_t))
        {
            IRQ_MASK_SET_L_Data |= (1u << 14);
            tmp = 0x20;
            uc_mem_write(MTK, 0x34002C28, &tmp, 4);
            {
                static clock_t os_timer_base_evt = 0;
                if (os_timer_base_evt == 0) os_timer_base_evt = clock();
                halTimerCount = (u32)((clock() - os_timer_base_evt) * 32768LL / CLOCKS_PER_SEC);
            }
            uc_mem_write(MTK, 0x34002C04, &halTimerCount, 4);
            uc_mem_write(MTK, 0x34002C08, &halTimerCount, 4);
            if (StartInterrupt(14, address))
            {
                timer_event_pending = 0;
                /*
                 * IRQ 13 (Rtk10TimeSoftHandler) 通过固件 HalCommonIntHandler
                 * 循环链式投递，避免嵌套中断导致 SPSR/T-bit 损坏。
                 * 设置掩码位和队列标记，hookRam 的 IRQ 清除钩子会在
                 * IRQ 14 处理完毕后将 IRQ 13 号码写入应答寄存器。
                 */
                if (soft_timer_event_pending)
                {
                    IRQ_MASK_SET_L_Data |= (1u << 13);
                    uc_mem_write(MTK, 0x3400181C, &IRQ_MASK_SET_L_Data, 4);
                    irq13_chained = 1;
                    soft_timer_event_pending = 0;
                }
            }
        }
    }

    /*
     * Pen detect 硬件级触发：
     * SDL 线程设置 pen_detect_pending=1（DOWN 事件），
     * 此处将 AUX_IRQ_STS bit0 置位并在 IRQ 使能时触发 IRQ 31。
     * 固件的 HalTSPenDetIrqHandler 自然检查 IRQ_STS & IRQ_EN，
     * 之后通过消息链触发 ADC 读取，完全走硬件路径。
     */
    if (pen_detect_pending)
    {
        aux_irq_sts |= 1;
        uc_mem_write(MTK, 0x3400C1C4u, &aux_irq_sts, 4);

        if (aux_irq_en & 1)
        {
            u32 cpsr_chk;
            uc_reg_read(MTK, UC_ARM_REG_CPSR, &cpsr_chk);
            if (!isIRQ_Disable(cpsr_chk))
            {
                IRQ_MASK_SET_L_Data |= (1u << 31);
                if (StartInterrupt(31, address))
                {
                    pen_detect_pending = 0;
                    auxadc_log_count = 0;
                }
            }
        }
    }

    if (handleTick++ > 100)
    {
        handleTick = 0;
        vmEvent = DequeueVMEvent();
        if (vmEvent > 0)
        {
            switch (vmEvent->event)
            {
            case VM_EVENT_MSDC_IRQ:
                if (!StartInterrupt(15 + 32, address))
                {
                    EnqueueVMEvent(VM_EVENT_MSDC_IRQ, 0, 0);
                    // printf("requeue mie %x\n", FICE_Status);
                }
                break;
            case VM_EVENT_Timer_IRQ:
            {
                tmp = 0x20;
                uc_mem_write(MTK, 0x34002C28, &tmp, 4);
                static clock_t os_timer_base_q = 0;
                if (os_timer_base_q == 0) os_timer_base_q = clock();
                halTimerCount = (u32)((clock() - os_timer_base_q) * 32768LL / CLOCKS_PER_SEC);
                uc_mem_write(MTK, 0x34002C04, &halTimerCount, 4);
                uc_mem_write(MTK, 0x34002C08, &halTimerCount, 4);
                IRQ_MASK_SET_L_Data |= (1u << vmEvent->r0);
                if (!StartInterrupt(vmEvent->r0, address))
                    EnqueueVMEvent(VM_EVENT_Timer_IRQ, vmEvent->r0, 0);
                break;
            }
            case VM_EVENT_KEYBOARD:
                handleKeyPadVmEvent(vmEvent, address);
                break;
            case VM_EVENT_LCD_IRQ:
                IRQ_MASK_SET_H_Data |= (1u << 19);
                if (StartInterrupt(19 + 32, address))
                {
                    tmp = 0xf << 8;
                    uc_mem_write(MTK, 0x74003148, &tmp, 4);
                    tmp = (1 << 3) | 1;
                    uc_mem_write(MTK, 0x7400314C, &tmp, 4);
                }
                break;
            case VM_EVENT_DMA_IRQ:
                IRQ_MASK_SET_H_Data |= (1u << 23);
                if (StartInterrupt(23 + 32, address))
                {
                    tmp = 0xffffffff;
                    uc_mem_write(MTK, 0x74000408, &tmp, 4);
                }
                else
                    EnqueueVMEvent(VM_EVENT_DMA_IRQ, 0, 0);
                break;
            case VM_EVENT_TOUCH_SCREEN_IRQ:
                break;
            case VM_EVENT_RTC_IRQ:
                IRQ_MASK_SET_L_Data |= (1u << DEV_IRQ_CHANNEL_RTC);
                if (!StartInterrupt(DEV_IRQ_CHANNEL_RTC, address))
                    EnqueueVMEvent(VM_EVENT_RTC_IRQ, 0, 0);
                break;
            case VM_EVENT_GPT_IRQ:
            {
                tmp = vmEvent->r0;
                uc_mem_write(MTK, 0x81060010, &tmp, 4);
                IRQ_MASK_SET_L_Data |= (1u << DEV_IRQ_CHANNEL_GPT);
                if (!StartInterrupt(DEV_IRQ_CHANNEL_GPT, address))
                    EnqueueVMEvent(VM_EVENT_GPT_IRQ, vmEvent->r0, 0);
                break;
            }
            case VM_EVENT_EXIT:
                uc_emu_stop(MTK);
                break;
            }
        }
    }
}
