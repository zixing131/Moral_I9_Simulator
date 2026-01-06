#include "vmEvent.h"
#define MAX_VM_EVENT_COUNT 512
#define MAX_VM_EVENT_WAIT_COUNT 512

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
            // 将等待队列的事件加入到处理队列中
            for (i = 0; i < VmEventWaitCount; i++)
            {
                // todo 剩余的还要回到等待队列
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

inline void handleVmEvent_EMU(uint64_t address)
{
    u32 tmp;
    if (handleTick++ > 1000)
    {
        handleTick = 0; // 防止事件处理过快
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
                // DrvTimerOstickGetCount
                // TimeoutLength [0x34002C04]
                // DrvTimerOstickIntFlag [0x34002C28]
                tmp = 0x20;
                uc_mem_write(MTK, 0x34002C28, &tmp, 4);
                uc_mem_read(MTK, 0x34002C04, &halTimerCount, 4);
                uc_mem_write(MTK, 0x34002C08, &halTimerCount, 4);
                StartInterrupt(vmEvent->r0, address);
                break;
            case VM_EVENT_KEYBOARD:
                // 写入按键状态
                // 假如6行 4列
                if (vmEvent->r1 == 1)
                {
                    // tmp = ((1 << keyRowIdx)) | (((1 << keyColIdx)) << 8);
                    tmp = ++keyRowIdx;
                    // if (keyRowIdx == 8)
                    // {
                    //     keyRowIdx = 0;
                    //     keyColIdx++;
                    //     if (keyColIdx == 8)
                    //         keyColIdx = 0;
                    // }
                    printf("Test Key Col:%d ,Row:%d ,tmp: %d\n", keyColIdx, keyRowIdx, tmp);
                }
                else
                    tmp = 0;
                uc_mem_write(MTK, 0x34000814, &tmp, 4);
                if (StartInterrupt(12, address))
                {
                    printf("key event ok \n");
                }
                else
                    EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);
                break;
            case VM_EVENT_LCD_IRQ:
                // 位于高32位中断
                if (StartInterrupt(19 + 32, address))
                {
                    tmp = 0xf << 8;
                    uc_mem_write(MTK, 0x74003148, &tmp, 4);
                    tmp = (1 << 3) | 1;
                    uc_mem_write(MTK, 0x7400314C, &tmp, 4);
                    EnqueueVMEvent(VM_EVENT_LCD_IRQ, 0, 0);
                }
                else
                    EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);
                break;
            case VM_EVENT_DMA_IRQ:
                // 位于高32位中断
                if (StartInterrupt(23 + 32, address))
                {
                    tmp = 0xffffffff;
                    uc_mem_write(MTK, 0x74000408, &tmp, 4);
                }
                else
                    EnqueueVMEvent(VM_EVENT_DMA_IRQ, 0, 0);
                break;
            case VM_EVENT_TOUCH_SCREEN_IRQ:
                if (vmEvent->r0 != 3)
                { // detect中断
                    if (StartInterrupt(31, address))
                    {
                        printf("handle touch down/up:%d\n", vmEvent->r0);
                        tmp = 3;
                        uc_mem_write(MTK, 0x3400C1BC, &tmp, 4);
                        tmp = 1;
                        uc_mem_write(MTK, 0x3400C1C4, &tmp, 4);
                    }
                    else
                        EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);
                }
                else
                { // adc采样中断
                    if (StartInterrupt(29, address))
                    {
                        printf("handle touch adc ");
                        tmp = vmEvent->r1 & 0xffff; // y
                        uc_mem_write(MTK, 0x3400C1C8, &tmp, 4);
                        printf("%d", tmp);
                        tmp = (vmEvent->r1 >> 16) & 0xffff; // x
                        uc_mem_write(MTK, 0x3400C1C0, &tmp, 4);
                        printf(",%d", tmp);
                        tmp = 2;
                        uc_mem_write(MTK, 0x3400C1C4, &tmp, 4);
                        printf("\n");
                    }
                    else
                        EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);
                }

                break;
            case VM_EVENT_EXIT:
                uc_emu_stop(MTK);
                break;
            }
        }
    }
}