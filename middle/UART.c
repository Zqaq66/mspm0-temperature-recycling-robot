#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "string.h"
#include "UART.h"
char time_buf[20];  // 存储接收的时间戳字符串
uint8_t time_idx = 0;        // 当前接收位置索引
uint8_t time_received = 0;   // 接收完成标志（0：未完成；1：已完成）
/*串口初始化*/
void UART_Init(void)
{
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}
/*串口发送字符*/
void UART_send_char(char ch)
{
	while(DL_UART_isBusy(UART0)==true);
	DL_UART_Main_transmitData(UART0,ch);
}
//串口的中断服务函数
void UART_0_INST_IRQHandler(void)
{
	uint8_t rx_char;
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_0_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            /* 收到字节累积；遇 '\n'/'' 结束一帧（补 '\0' 并置完成标志）
             * 原实现：'\n' 会走 else 分支、帧永远不结束；20 字节上界也偏紧 */
            rx_char = DL_UART_Main_receiveData(UART_0_INST);
            if (rx_char == '\n' || rx_char == '\r') {
                time_buf[time_idx] = '\0';
                time_received      = 1;
                time_idx           = 0;
            } else if (time_idx < sizeof(time_buf) - 1) {
                time_buf[time_idx++] = (char)rx_char;
            }
            break;

        default://其他的串口中断
            break;
	}
    } 
/*Printf重定向*/
int fputc(int ch, FILE *stream)
{
        while( DL_UART_isBusy(LORA_INST) == true );

        DL_UART_Main_transmitData(LORA_INST, ch);
//		 while( DL_UART_isBusy(UART_0_INST) == true )
//			 ;
//		 DL_UART_Main_transmitData(UART_0_INST, ch);

        return ch;
}

#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
        int handle;
};
#endif
FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
        x = x;
}
#endif