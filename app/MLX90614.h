#ifndef __MLX96014_
#define __MLX96014_
#include "ti_msp_dl_config.h"
#include "Time.h"

/* 说明：PA0/PA1 已交给硬件 I2C0（上挂 Xinliu_AuroraT8 循迹 0x40 与本模块 0x5A；
 *       JY61P 是 IMU，走 UART1(PA8/PA9)，不在这条 I2C 总线上）；
 *       本模块底层已改为硬件 I2C 读写，见 MLX90614.c；
 *       原先的 GPIO 模拟时序(SDA/SCL/SDA_OUT...)已删除。
 * 用法：float t = MLX90614_Read(0x5A, 0x07);   // 0x07 = 物体温度(℃)，读失败返回 0.0f */

float MLX90614_Read(unsigned char SlaveAddr, unsigned char RegAddr);
//MLX90614_Read(0X5A, 0X07)
#endif
