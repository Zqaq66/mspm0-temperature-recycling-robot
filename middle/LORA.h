/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-02     LCKFB-LP    first version
 */

#ifndef _BSP_LORA_H_
#define _BSP_LORA_H_
#include "string.h"  
#include "Time.h"
extern unsigned char LOAR_RX_BUFF[300]; 
/****************************   串口配置    ****************************/
#define RCC_LORA_TX                 RCC_APB2Periph_GPIOA   // 串口TX的端口时钟
#define RCC_LORA_RX                 RCC_APB2Periph_GPIOA   // 串口RX的端口时钟
#define RCC_LORA_USART              RCC_APB1Periph_USART2  // 串口2的时钟

#define PORT_LORA_TX                GPIOA               // 串口TX的端口
#define PORT_LORA_RX                GPIOA               // 串口RX的端口

#define GPIO_LORA_TX                GPIO_Pin_8               // 串口TX的引脚
#define GPIO_LORA_RX                GPIO_Pin_18              // 串口RX的引脚

#define LORA_USART                  LORA_INST                  // 串口2
#define LORA_USART_IRQ              LORA_INST_INT_IRQN              // 串口2中断
#define LORA_USART_IRQHandler       LORA_INST_IRQHandler        // 串口2中断服务函数


#define     LOAR_RX_LEN_MAX     300       //串口接收最大长度

void LOAR_USART_Init(void);
void LOAR_USART_send_String(unsigned char *str);
void LOAR_USART_send_HEX(unsigned char *str, unsigned int len);
unsigned char Anakysis_Data(void);
#endif

