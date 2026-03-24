#pragma once

#include "main.h"

#define GPT_BASE 0x81060000
// &0x8000 = 0 停用 &0x8000 = 1 启用
// &0x4000 = 0 单次 &0x4000 = 1 循环
#define GPT1_CONTROL (GPT_BASE + 0x00)
#define GPT1_TOUT_INTERVAL (GPT_BASE + 0x04)
#define GPT2_CONTROL (GPT_BASE + 0x08)
#define GPT2_TOUT_INTERVAL (GPT_BASE + 0x0C)
// &0x1 = 0 未超时 &0x1 = 1 超时
// 第几位超时，就对应那个定时器
#define GPT_STAT (GPT_BASE + 0x10)

// 低三位有效 基础16Mhz，值为n就表示分频为：16M/2^n
#define GPT1_PRESCALER (GPT_BASE + 0x14)
#define GPT2_PRESCALER (GPT_BASE + 0x18)


uint64_t last_gpt1_interrupt_time;
uint64_t last_gpt2_interrupt_time;
bool gpt1Enable;
bool gpt2Enable;
bool gpt1Circle;
bool gpt2Circle;
uint64_t gpt1Tout;
uint64_t gpt2Tout;