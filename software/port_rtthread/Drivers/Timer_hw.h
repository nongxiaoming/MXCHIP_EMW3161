#ifndef _TIMER_H_
#define _TIMER_H_
#include "stm32f2xx.h"

void Timer_Sys_Init(u32 period_num);
void Timer_Sys_Run(u8 ena);

#endif
