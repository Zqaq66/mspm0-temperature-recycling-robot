#include "ti_msp_dl_config.h"
#include "LORA.h"
#include "stdio.h"
#include "string.h"


unsigned char LOAR_RX_BUFF[LOAR_RX_LEN_MAX];     
unsigned char LOAR_RX_FLAG = 0;
unsigned char LOAR_RX_LEN = 0;

/************************************************************
 * 函数名称：LOAR_USART_Init
 * 函数说明：连接LOAR的初始化
 * 型    参：bund=串口波特率
 * 返 回 值：无
 * 备    注：波特率根据你设置的内容来决定(默认波特率9600)
*************************************************************/
void LOAR_USART_Init(void)
{
        NVIC_ClearPendingIRQ(LORA_INST_INT_IRQN);
		NVIC_EnableIRQ(LORA_INST_INT_IRQN);
}

/******************************************************************
 * 函 数 名 称：LOAR_USART_Send_Bit
 * 函 数 说 明：向LOAR模块发送单个字符
 * 函 数 形 参：ch=字符
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void LOAR_USART_Send_Bit(unsigned char ch)
{
	DL_UART_Main_transmitData(UART1,ch);    
    while(DL_UART_isBusy(UART1)==true);
}  


/******************************************************************
 * 函 数 名 称：LOAR_USART_send_String
 * 函 数 说 明：向LOAR模块发送字符串
 * 函 数 形 参：str=发送的字符串
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void LOAR_USART_send_String(unsigned char *str)
{
        while( str && *str ) // 地址为空或者值为空跳出
        {        
                LOAR_USART_Send_Bit(*str++);
        }        
}

/******************************************************************
 * 函 数 名 称：LOAR_USART_send_String
 * 函 数 说 明：向LOAR模块发送字符串
 * 函 数 形 参：str=发送的16进制数据        len=发送的数据长度
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void LOAR_USART_send_HEX(unsigned char *str, unsigned int len)
{
        while( len-- ) // 地址为空或者值为空跳出
        {        
                LOAR_USART_Send_Bit(*str++);
        }        
}

/******************************************************************
 * 函 数 名 称：Clear_LOAR_RX_BUFF
 * 函 数 说 明：清除LOAR发过来的数据
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void Clear_LOAR_RX_BUFF(void)
{
        unsigned int i = LOAR_RX_LEN_MAX - 1;
        while(i)        
        {
                LOAR_RX_BUFF[i--] = 0;
        }
        LOAR_RX_LEN = 0;
        LOAR_RX_FLAG = 0;
}

/**********************************************************
 * 函 数 名 称：Anakysis_Data
 * 函 数 功 能：接收LoRa发送过来的数据
 * 传 入 参 数：无
 * 函 数 返 回：1=接收到一次数据                0=当前没有接收到数据
 * 作       者：LC
 * 备       注：
**********************************************************/
unsigned char Anakysis_Data(void)
{
        if( LOAR_RX_FLAG == 1 )//如果接收到串口的数据
        {
                LOAR_RX_FLAG = 0;//将标志位清除
//                printf("data = %s\r\n",LOAR_RX_BUFF);
                Clear_LOAR_RX_BUFF();
                return 1;
        }
//                else
//                {
//                        printf("NO Data!\r\n");
//                }
        return 0;
}


/******************************************************************
 * 函 数 名 称：LOAR_USART_IRQHandler
 * 函 数 说 明：连接LOAR的串口中断服务函数
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void LORA_USART_IRQHandler(void)
{
//        if(USART_GetITStatus(LORA_USART,USART_IT_RXNE) != RESET) // 接收缓冲区不为空
//        {
//        //接收数据
//                LOAR_RX_BUFF[ LOAR_RX_LEN ] = USART_ReceiveData(LORA_USART);
//        
//#if DEBUG
//        //测试，查看接收到了什么数据
//        printf("%c", LOAR_RX_BUFF[ LOAR_RX_LEN ]);
//#endif
//        
//                //接收长度限制
//        LOAR_RX_LEN = ( LOAR_RX_LEN + 1 ) % LOAR_RX_LEN_MAX;
//                        
//                USART_ClearFlag(LORA_USART,USART_FLAG_RXNE);//初始配置时清除接受置位
//        }
//        if(USART_GetITStatus(LORA_USART,USART_IT_IDLE) == SET) // 检测到空闲中断
//        {
//                                volatile uint32_t temp;
//                temp = LORA_USART->SR; // 读取状态寄存器以清除IDLE标志
//                temp = LORA_USART->DR; // 读取数据寄存器以清除IDLE标志
//                        
//                LOAR_RX_BUFF[LOAR_RX_LEN] = '\0'; //字符串结尾补 '\0'
//                LOAR_RX_FLAG = SET;            // 接收完成
//                        
//                                //usart_interrupt_flag_clear(LOAR_USART, USART_INT_FLAG_IDLE);
//        }
 switch( DL_UART_getPendingInterrupt(LORA_USART) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            /* 应用层约定：读 LOAR_RX_BUFF[0] 取"最新一个字节"。
             * 原实现 LOAR_RX_BUFF[LOAR_RX_LEN++] 无边界检查（收满 300 字节越界），
             * 且紧接着 printf("%c", ...[LOAR_RX_LEN]) 越界读 1 字节、还在中断里阻塞发串（9600 下约 3ms）。
             * 现改为只保留最新字节：语义不变、无越界；将来要收多字节命令再上环形缓冲+帧尾判定。 */
            LOAR_RX_BUFF[0] = DL_UART_Main_receiveData(LORA_USART);
            LOAR_RX_LEN     = 1;
            break;
        default://其他的串口中断(帧错误/溢出/超时等)
            /* 读一次数据寄存器清掉错误条件; 不清的话噪声会在中断里反复触发 */
            (void)DL_UART_Main_receiveData(LORA_USART);
            break;
    }
}

