#ifndef _BSP_LORA_H_
#define _BSP_LORA_H_

/* 433M LORA (UART3; TX=PB2 / RX=PB3, 9600-8-N-1, 引脚见 ti_msp_dl_config.h 的 LORA_INST)
 *   发送: printf 直接走 UART.c 的 fputc -> LORA_INST; 本文件另给 字符串/HEX 发送
 *   接收: LORA_USART_IRQHandler 只保留最新字节在 LOAR_RX_BUFF[0]
 *   ★ 原来的 STM32 宏 (RCC_LORA_xx / PORT_LORA_xx / GPIO_LORA_xx) 是教程残留, 已删除 */

#include "string.h"
#include "Time.h"

#define LOAR_RX_LEN_MAX     300
extern unsigned char LOAR_RX_BUFF[LOAR_RX_LEN_MAX];

#define LORA_USART              LORA_INST
#define LORA_USART_IRQ          LORA_INST_INT_IRQN
#define LORA_USART_IRQHandler   LORA_INST_IRQHandler

void LOAR_USART_Init(void);
void LOAR_USART_send_String(unsigned char *str);
void LOAR_USART_send_HEX(unsigned char *str, unsigned int len);
unsigned char Anakysis_Data(void);

#endif
