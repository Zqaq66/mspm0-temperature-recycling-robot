#include "software_iic.h"
#include "Time.h"
#define ACK 0x0 // acknowledge (SDA LOW)
#define NACK 0x1 // not acknowledge (SDA HIGH)

#define LOW 0x0
#define HIGH 0x1

#define I2C_READ 0x1
#define I2C_WRITE 0x0
/* 基本时序操作 */
static void IIC_Delay(void)
{
    delay_us(10);  // 根据实际I2C速度调整延时
}
/* 软件I2C基础函数 */



void IIC_Start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    IIC_Delay();
    SDA_LOW();
    IIC_Delay();
    SCL_LOW();
}

void IIC_Stop(void)
{
    SDA_LOW();
    IIC_Delay();
    SCL_HIGH();
    IIC_Delay();
    SDA_HIGH();
    IIC_Delay();
}

uint8_t IIC_WaitAck(void)
{
    uint8_t ack;
    SDA_HIGH();
    SCL_HIGH();
    IIC_Delay();
    ack = READ_SDA();
    SCL_LOW();
    IIC_Delay();
    return ack;
}

void IIC_SendAck(void)
{
    SDA_LOW();
    SCL_HIGH();
    IIC_Delay();
    SCL_LOW();
    SDA_HIGH();
}

void IIC_SendNAck(void)
{
    SDA_HIGH();
    SCL_HIGH();
    IIC_Delay();
    SCL_LOW();
}

uint8_t IIC_SendByte(uint8_t dat)
{
    for(uint8_t i = 0; i < 8; i++) {
        (dat & 0x80) ? SDA_HIGH() : SDA_LOW();
        dat <<= 1;
        SCL_HIGH();
        IIC_Delay();
        SCL_LOW();
        IIC_Delay();
    }
    return IIC_WaitAck();
}

uint8_t IIC_RecvByte(void)
{
    uint8_t dat = 0;
    SDA_HIGH();
    /* 接收数据前切换SDA为输入模式 */
		DL_GPIO_initDigitalInput(Software_iic_SDA_IOMUX);
    for(uint8_t i = 0; i < 8; i++) {
        dat <<= 1;
        SCL_HIGH();
        IIC_Delay();
        if(READ_SDA()) dat |= 0x01;
        SCL_LOW();
        IIC_Delay();
    }
		DL_GPIO_initDigitalOutput(Software_iic_SDA_IOMUX);    
    DL_GPIO_setPins(SDA_PORT, SDA_PIN);      
    DL_GPIO_enableOutput(SDA_PORT, SDA_PIN); 

    return dat;
}

/* 应用层函数改写 */
uint8_t IIC_ReadByte(uint8_t Salve_Address)
{
    uint8_t dat;
    
    IIC_Start();
    IIC_SendByte(Salve_Address | 0x01);  // 读模式
    dat = IIC_RecvByte();
    IIC_SendNAck();
    IIC_Stop();
    
    return dat;
}

uint8_t IIC_ReadBytes(uint8_t Salve_Address, uint8_t Reg_Address, 
                          uint8_t *Result, uint8_t len)
{
    IIC_Start();
    if(IIC_SendByte(Salve_Address & 0xFE)) {  // 写模式
        IIC_Stop();
        return 0;
    }
    if(IIC_SendByte(Reg_Address)) {
        IIC_Stop();
        return 0;
    }
    IIC_Start();
    if(IIC_SendByte(Salve_Address | 0x01)) {  // 读模式
        IIC_Stop();
        return 0;
    }
    
    for(uint8_t i = 0; i < len; i++) {
        Result[i] = IIC_RecvByte();
        (i == len-1) ? IIC_SendNAck() : IIC_SendAck();
    }
    IIC_Stop();
    return 1;
}

uint8_t IIC_WriteByte(uint8_t Salve_Address, uint8_t Reg_Address, 
                          uint8_t data)
{
    IIC_Start();
    if(IIC_SendByte(Salve_Address & 0xFE)) {  // 写模式
        IIC_Stop();
        return 0;
    }
    if(IIC_SendByte(Reg_Address)) {
        IIC_Stop();
        return 0;
    }
    if(IIC_SendByte(data)) {
        IIC_Stop();
        return 0;
    }
    IIC_Stop();
    return 1;
}

uint8_t IIC_WriteBytes(uint8_t Salve_Address, uint8_t Reg_Address,
                           uint8_t *data, uint8_t len)
{
    IIC_Start();
    if(IIC_SendByte(Salve_Address & 0xFE)) {
        IIC_Stop();
        return 0;
    }
    if(IIC_SendByte(Reg_Address)) {
        IIC_Stop();
        return 0;
    }
    
    for(uint8_t i = 0; i < len; i++) {
        if(IIC_SendByte(data[i])) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 1;
}
uint8_t Ping(void)
{
	uint8_t dat;
	IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,GW_GRAY_PING,&dat,1);
	if(dat==GW_GRAY_PING_OK)
	{
			return 0;
	}	
	else return 1;
}

