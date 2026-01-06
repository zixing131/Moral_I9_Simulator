#include "main.h"
#include "vmEvent.h"

#define RTC_ISR_STATE_REG 0x810b0000
#define RTC_SECOND_REG 0x810b0014
#define RTC_MINUTE_REG 0x810b0018
#define RTC_HOUR_REG 0x810b001C
#define RTC_DAY_REG 0x810b0020
#define RTC_WEEKDAY_REG 0x810b0024
#define RTC_MONTH_REG 0x810b0028
#define RTC_YEAR_REG 0x810b002C

clock_t last_rtc_interrupt_time;

void RtcTaskMain();
void handleRtcVmEvent(vm_event *vmEvent, uint64_t address);