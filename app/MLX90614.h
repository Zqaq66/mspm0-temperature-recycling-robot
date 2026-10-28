#ifndef __MLX90614_H
#define __MLX90614_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*===========================================================================
 * MLX90614 / GY-906 非接触红外测温驱动 (本工程版)
 *---------------------------------------------------------------------------
 * 例程来源: "GY906红外温度模块" 例程 (stm32f4_MLX90614 / GY906_IPS240 版)。
 *   例程用 GPIO 模拟 SMBus 时序, PB6=SCL / PB7=SDA。
 *
 * 本工程直接用【软件模拟 I2C】(和例程同一套路子), 但把引脚换成本板上可用的空脚:
 *   SCL = PB17, SDA = PB18
 *
 * 【接线】MLX90614 SCL -> PB17   SDA -> PB18   VIN -> 3V3   GND -> GND(共地)
 *   ★ 不复用循迹那条 I2C0 总线: PA0/PA1 保持给灰度传感器, 两边互不影响, 可以同时接。
 *
 * 【地址】7 位地址 0x5A。例程里个别版本写 0x00, 那只在"单器件 + 某些克隆模块"上碰巧
 *   能用; 上电时 0x5A / 0x00 都会试一遍。
 *
 * 【协议】沿用例程的 SMBus 读字序列 + 标准 CRC-8 校验 PEC(例程那个 PEC 算法是错的):
 *   S | addr+W | cmd | Sr | addr+R | DataL | DataH | PEC | P
 *   温度 = raw * 0.02 - 273.15
 *
 * 【用法】
 *     float t = MLX90614_ReadTemp();          // 物体温度(℃), 失败返回 MLX90614_ERR
 *     float t = MLX90614_Read(0x5A, 0x07);    // 兼容旧接口, 失败返回 0.0f
 *   寄存器: 0x06 = 环境温度(Tamb)   0x07 = 物体温度(Tobj1, 测温用这个)
 *===========================================================================*/

#ifndef MLX90614_I2C_ADDR
#define MLX90614_I2C_ADDR    0x5Au    /* 7 位从机地址 (可用 -DMLX90614_I2C_ADDR=0x00 试别的地址) */
#endif
#define MLX90614_REG_TAMB    0x06u    /* 环境温度 */
#define MLX90614_REG_TOBJ1   0x07u    /* 物体温度 (默认测温用) */

#ifndef MLX90614_RETRY
#define MLX90614_RETRY       4        /* 单次读取最多尝试几遍 (PEC 不符就重读) */
#endif
#ifndef MLX90614_CHECK_PEC
#define MLX90614_CHECK_PEC   1        /* 1 = 读 3 字节并校验 PEC; 0 = 只读 2 字节 (旧行为) */
#endif

#define MLX90614_ERR         (-1000.0f)  /* 读取失败的哨兵值 (远离任何真实温度) */

/* 读一个 16 位字 (SMBus Read Word + PEC 校验 + 重试)
 *   返回: 0 = 成功且 PEC 正确
 *         1 = 读到了数据但 PEC 始终不符 (返回最后一次数据, 并计一次 PEC 错)
 *        <0 = 总线失败, 没读到数据;  raw 可为 NULL */
int   MLX90614_ReadWord(unsigned char SlaveAddr, unsigned char RegAddr, uint16_t *raw);

/* 物体温度 (寄存器 0x07), 摄氏度; 读取失败返回 MLX90614_ERR */
float MLX90614_ReadTemp(void);

/* 兼容旧接口 (empty.c 的 Temp_ReadBody 还在用): 读寄存器并转 ℃; 失败返回 0.0f */
float MLX90614_Read(unsigned char SlaveAddr, unsigned char RegAddr);

/* 标准 SMBus CRC-8 (多项式 0x07) —— MLX90614 的 PEC 算法 */
unsigned char MLX90614_CRC8(const unsigned char *data, unsigned char len);

/* 诊断: 探测一个 7 位地址在总线上有没有应答 (只发地址+读 1 字节, 不向设备写数据)
 *   返回 0 = 有应答 (ACK);  <0 = 没应答 (NACK)
 *   用来区分 "TEMP --.-C" 是【设备没应答】(接线/供电/上拉/速度/地址) 还是
 *   【应答了但读时序不对】(驱动/PEC 问题)。 */
int MLX90614_ProbeAddr(unsigned char addr7);

/* 诊断: 扫描 I2C0 总线, 找出所有有应答的 7 位地址 (对每个地址做一次真实的读事务)。
 *   回答"是不是 I2C 地址不对": 扫到的地址会存进 list 并返回个数。
 *   ★ 会向每个地址(若有器件)发一次命令字 0x07 —— 本板总线上只有 MLX90614/灰度,
 *     这一次探测对它们无副作用。返回找到的个数; list 可为 NULL。 */
int MLX90614_BusScan(unsigned char *list, int maxn);

/* 诊断: 上电以来"读到了数据但 PEC 校验失败"的次数 */
uint32_t MLX90614_GetPecErrCount(void);

/* 上电自动探测读时序: 先试"带 STOP", 再试"重复起始", 谁能读到温度就用谁。
 * (MLX90614 手册要求重复起始, 但实测有些模块用带 STOP 的方式也能读;
 *  自动探测省得手工改宏。) 建议在 System_Init 里调一次。 */
void MLX90614_AutoDetect(void);

/* 诊断: 自动探测出来的有效从机地址 (默认 0x5A; 厂商例程那种模块可能是 0x00) */
unsigned char MLX90614_GetAddr(void);

/* 初始化: 软 I2C 时把 PB17(SCL)/PB18(SDA) 配成 GPIO (硬件 I2C 时什么都不做)。上电调一次。
 * ★ 这两个脚是独立的空脚, 和灰度(I2C0 PA0/PA1) 分开, 两者可以同时用。 */
void MLX90614_Init(void);

#endif /* __MLX90614_H */
