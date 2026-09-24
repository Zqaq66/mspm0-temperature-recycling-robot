/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 文档网站：wiki.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 嘉立创社区问答：https://www.jlc-bbs.com/lckfb
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 */
#ifndef	__BSP_GYRO_H__
#define __BSP_GYRO_H__

#include "ti_msp_dl_config.h"
#include "Time.h"
#include "UART.h"
#include "MLX90614.h"
// 调试开关
#define GYRO_DEBUG	0

// 定义一个结构体来存储
typedef struct {
    float x;
    float y;
    float z;
} Gyro_Struct;

// 模块地址
#define	IIC_ADDR		0x50
// 航向角地址
#define YAW_REG_ADDR	0x3F
// 寄存器解锁
#define UN_REG			0x69
// 保存寄存器
#define SAVE_REG		0x00
// 角度参考寄存器
#define ANGLE_REFER_REG	0x01

/* 注意：PA0/PA1 已由硬件 I2C0 接管（循迹传感器 + MLX90614 共用同一条总线）。
 * 本模块(JY61P)当前未接入主流程，下面这段模拟时序只为保证编译通过，实际不可用；
 * 若要在车上使用 JY61P，请改走 DL_I2C 硬件接口（参见 MLX90614.c 的写法）。 */
//SDA输入模式
#define SDA_IN()   {  DL_GPIO_initDigitalInput(GPIO_I2C_0_IOMUX_SDA); }
//SDA输出模式
#define SDA_OUT()  {  DL_GPIO_initDigitalOutput(GPIO_I2C_0_IOMUX_SDA); \
                      DL_GPIO_enableOutput(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN); \
                   }

#define SCL(BIT)  ( BIT ? DL_GPIO_setPins(GPIO_I2C_0_SCL_PORT,GPIO_I2C_0_SCL_PIN) : DL_GPIO_clearPins(GPIO_I2C_0_SCL_PORT,GPIO_I2C_0_SCL_PIN) )
#define SDA(BIT)  ( BIT ? DL_GPIO_setPins(GPIO_I2C_0_SDA_PORT,GPIO_I2C_0_SDA_PIN) : DL_GPIO_clearPins(GPIO_I2C_0_SDA_PORT,GPIO_I2C_0_SDA_PIN) )
#define SDA_GET() ( ( DL_GPIO_readPins( GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN ) & GPIO_I2C_0_SDA_PIN ) ? 1 : 0 )

void jy61pInit(void);
uint8_t readDataJy61p(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
uint8_t writeDataJy61p(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
Gyro_Struct *get_angle(void);

#endif