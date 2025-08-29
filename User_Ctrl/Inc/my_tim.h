#ifndef _MY_TIM
#define _MY_TIM

#include "main.h"
#include "tim.h"

extern uint8_t timer_1ms_Flag;
extern uint8_t timer_5ms_Flag;
extern uint32_t beep_Cnt;

void TIM_ENABLE(void);

#endif
