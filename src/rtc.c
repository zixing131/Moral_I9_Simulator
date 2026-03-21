#include "rtc.h"

void Update_RTC_Time()
{
    time_t now = time(NULL);
    struct tm *local_time = localtime(&now);

    uc_mem_read(MTK, RTC_ISR_STATE_REG, &changeTmp1, 4);
    if (changeTmp1 != 2)
    {
        changeTmp1 = 2; // 2表示计数器中断 1表示闹钟中断
        uc_mem_write(MTK, RTC_IRQ_STATUS, &changeTmp1, 4);
        changeTmp1 = 0; // 只有秒=0时才会触发更新
    }
    else
    {
        changeTmp1 = local_time->tm_sec; // 只有秒=0时才会触发更新
    }
    uc_mem_write(MTK, RTC_SECOND_REG, &changeTmp1, 4); // 秒
    changeTmp1 = local_time->tm_min;
    uc_mem_write(MTK, RTC_MINUTE_REG, &changeTmp1, 4); // 分
    changeTmp1 = local_time->tm_hour;
    uc_mem_write(MTK, RTC_HOUR_REG, &changeTmp1, 4); // 时
    changeTmp1 = local_time->tm_mday;
    uc_mem_write(MTK, RTC_DAY_REG, &changeTmp1, 4); // 日
    changeTmp1 = local_time->tm_wday;
    uc_mem_write(MTK, RTC_WEEKDAY_REG, &changeTmp1, 4); // 星期
    changeTmp1 = local_time->tm_mon;
    uc_mem_write(MTK, RTC_MONTH_REG, &changeTmp1, 4); // 月
    changeTmp1 = local_time->tm_year - 100;           // 手机系统时间是从2000年开始， 时间修正
    uc_mem_write(MTK, RTC_YEAR_REG, &changeTmp1, 4);  // 年
}

inline void handleRtcVmEvent(vm_event *vmEvent, uint64_t address)
{
    if (vmEvent->event == VM_EVENT_RTC_IRQ)
    {
        Update_RTC_Time();
        StartInterrupt(DEV_IRQ_CHANNEL_RTC, address);
    }
}

void RtcTaskMain()
{
    if (currentTime > last_rtc_interrupt_time)
    {
        last_rtc_interrupt_time = currentTime + 500;
        Update_RTC_Time();
        EnqueueVMEvent(VM_EVENT_RTC_IRQ, 0, 0);
    }
}