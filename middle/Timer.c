/*===========================================================================
 * 1ms 系统时基实现 —— systick_counter 由 TIMER_0 中断累加, Get_Time() 读它
 *   Timer_Init/Start/Stop: 开/关定时器与其中断 (Start/Stop 会一起管 NVIC)
 *   APP_SysTick_Hook():    1ms 中断里的弱钩子, 默认空
 *===========================================================================*/

#include "Timer.h"

/* 1ms 递增的系统时间戳 (中断里改, 读的地方要注意是 volatile) */
volatile uint32_t systick_counter = 0;

/* 用户钩子: 每 1ms 在中断上下文里被调用一次, 默认空 (可在 empty.c 里重写) */
__attribute__((weak)) void APP_SysTick_Hook(void) {}

/*定时器初始化*/
void Timer_Init(void)
{
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}
/*开启定时器*/
void Timer_Start(void)
{
    DL_TimerG_startCounter(TIMER_0_INST);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}
/*关闭定时器*/
void Timer_Stop(void)
{
    DL_TimerG_stopCounter(TIMER_0_INST);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
}

/* TIMER_0 (TIMG6) 中断: 周期 1ms, LOAD = 1ms*10MHz-1 */
void TIMER_0_INST_IRQHandler(void)
{
	switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST))
	{
		case DL_TIMER_IIDX_ZERO:
			systick_counter++;      /* 每 1ms +1 */
			APP_SysTick_Hook();
			break;

		default:
			break;
	}
}

uint32_t Get_Time(void)
{
	return systick_counter;
}
