#include "ti_msp_dl_config.h"
#include "UART.h"
#include "stdio.h"
//TW_0:PB17
//TW_1:PB18
void TW_Init(void)
{
	DL_GPIO_clearPins(TW_PORT,TW_TW_0_PIN);
	DL_GPIO_clearPins(TW_PORT,TW_TW_1_PIN);
}
void Voice_0(char tem)
{
	if(tem==1)
	{
		 UART_send_char('E');
	}
}
void Voice_1(char tem)
{
	if(tem==1)
	{
		UART_send_char('F');
	}
}