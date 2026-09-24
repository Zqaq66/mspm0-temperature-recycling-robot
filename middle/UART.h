#ifndef __UART_
#define __UART_
#include "ti_msp_dl_config.h"
extern char time_buf[20];  // 存储接收的时间戳字符串
extern uint8_t time_received; 
void UART_send_char(char ch);
void UART_send_string(char *str);
void UART_send_Data(int tp);
void UART_Init();
#endif