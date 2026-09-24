#ifndef __MLX96014_
#define __MLX96014_
#include "ti_msp_dl_config.h"
#include "Time.h"

/* 说明：PA0/PA1 已交给硬件 I2C0（Xinliu_AuroraT8 循迹 + 本模块 + JY61P 共用一条总线），
 *       本模块底层已改为硬件 I2C 读写，见 MLX90614.c；
 *       原先的 GPIO 模拟时序(SDA/SCL/SDA_OUT...)已删除。 */

float MLX90614_Read(unsigned char SlaveAddr, unsigned char RegAddr);
//MLX90614_Read(0X5A, 0X07)
#endif
