#include "MLX90614.h"

#include "Time.h"          /* delay_us() (BSP/Time.h) */

/*===========================================================================
 * MLX90614 / GY-906 非接触红外测温驱动 —— 【软件模拟 I2C】版
 *---------------------------------------------------------------------------
 * 协议移植自 "GY906红外温度模块" 例程 (stm32f4_MLX90614 / GY906_IPS240 版):
 *   写地址+W -> 写命令 -> 【重复起始】 -> 地址+R -> 读 DataL/DataH/PEC -> 校验 -> 重试
 *   温度 = raw(0x07) * 0.02 - 273.15
 * 底层和例程一样: 用两个普通 GPIO 手动打 SMBus 时序。
 *
 * 【接线】(★ 不复用循迹那条 I2C0 总线)
 *     MLX90614 SCL -> PB17    SDA -> PB18    VIN -> 3V3    GND -> GND
 *   PA0/PA1 保持给循迹灰度传感器 (硬件 I2C0), 两边互不影响, 可以同时接。
 *
 * 【地址】7 位地址 0x5A (器件手册默认)。例程里 stm32f1/f4 两个版本写的是 0x00
 *   (那个不合 I2C 规范, 只在个别模块上碰巧能用), 所以上电时 0x5A / 0x00 都试一遍。
 *
 * 【PEC】例程的 PEC_Calculation() 是"48 位大端比特流 + 逐步移位异或"的写法,
 *   实测与标准 CRC-8 (poly=0x07, init=0x00, MSB-first) 约 50% 不吻合 —— 不是标准
 *   PEC。例程靠 SMBus_ReadMemory() 的 do-while 在 PEC 一直不符时把 ErrorCounter
 *   减到 0 再 break、照样把数据返回, 所以 PEC 形同虚设。本工程用标准 CRC-8。
 *===========================================================================*/

/* ---- 软件模拟 I2C 用的两个脚 (独立空脚, 不占 I2C0 的 PA0/PA1) ----
 *   PB17 = SCL  (GPIOB.17, IOMUX_PINCM43, 封装 pin 14)
 *   PB18 = SDA  (GPIOB.18, IOMUX_PINCM44, 封装 pin 15)
 *   这两个脚在 SysConfig 里本来就是 GPIO 输出(原预留语音模块), 换用途不影响别的外设。 */
#define MLX_SCL_PORT      GPIOB
#define MLX_SCL_PIN       DL_GPIO_PIN_17
#define MLX_SCL_PINCM     IOMUX_PINCM43
#define MLX_SDA_PORT      GPIOB
#define MLX_SDA_PIN       DL_GPIO_PIN_18
#define MLX_SDA_PINCM     IOMUX_PINCM44
#define MLX_SOFT_HALF_US  6u      /* 半位延时 us -> 约 80kHz (和例程一个量级) */

static uint32_t      mlx_pec_err = 0;                   /* PEC 校验失败次数 (诊断) */
static unsigned char mlx_addr    = MLX90614_I2C_ADDR;   /* 当前有效的从机地址 */

/*===========================================================================
 * 底层: GPIO 手动打 I2C 时序
 *   SCL = PB17: 推挽输出 (总线上只有主机驱动 SCL)
 *   SDA = PB18: 开漏 -> 用"输出低 / 切成输入放开"模拟, 高电平靠上拉
 *=========================================================================*/
static void scl_h(void) { DL_GPIO_setPins(MLX_SCL_PORT, MLX_SCL_PIN); }
static void scl_l(void) { DL_GPIO_clearPins(MLX_SCL_PORT, MLX_SCL_PIN); }

static void sda_low(void)
{
    DL_GPIO_initDigitalOutput(MLX_SDA_PINCM);
    DL_GPIO_enableOutput(MLX_SDA_PORT, MLX_SDA_PIN);
    DL_GPIO_clearPins(MLX_SDA_PORT, MLX_SDA_PIN);
}

/* 放开 SDA = 关掉输出驱动 + 切成输入; 打开【内部上拉】把电平拉高。
 * ★★ 这里必须先 disableOutput: 光把脚切成输入(PINCM)是【不够】的 —— GPIO 的
 *    输出使能位(DOE)是独立寄存器, 不关掉的话引脚仍然被驱动着拉低 (SDA 恒低 ->
 *    ACK 永远读成 0 -> 扫描把每个地址都当成"有应答")。这个坑踩过一次。
 * ★ 内部上拉约 40k, 80kHz 下偏弱: 读数不稳就把 MLX_SOFT_HALF_US 加大,
 *   或在面包板上给 PB18 -> 3V3 外接一个 4.7k 上拉。 */
static void sda_release(void)
{
    DL_GPIO_disableOutput(MLX_SDA_PORT, MLX_SDA_PIN);      /* 先松开输出驱动 */
    DL_GPIO_initDigitalInputFeatures(MLX_SDA_PINCM, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

static unsigned sda_in(void)
{
    return DL_GPIO_readPins(MLX_SDA_PORT, MLX_SDA_PIN) ? 1u : 0u;
}

static void bit_dly(void) { delay_us(MLX_SOFT_HALF_US); }

/* 起始条件: SCL 高时 SDA 由高变低 */
static void i2c_start(void)
{
    sda_release(); bit_dly();
    scl_h();       bit_dly();
    sda_low();     bit_dly();
    scl_l();       bit_dly();
}

/* 停止条件: SCL 高时 SDA 由低变高 */
static void i2c_stop(void)
{
    sda_low();     bit_dly();
    scl_h();       bit_dly();
    sda_release(); bit_dly();
}

static void send_bit(unsigned b)
{
    if (b) sda_release(); else sda_low();
    bit_dly();
    scl_h(); bit_dly();
    scl_l(); bit_dly();
}

static unsigned recv_bit(void)
{
    unsigned v;
    sda_release(); bit_dly();
    scl_h();       bit_dly();
    v = sda_in();
    scl_l();       bit_dly();
    return v;
}

/* 发一字节, 返回应答位 (0 = ACK, 1 = NACK) */
static unsigned send_byte(unsigned char b)
{
    unsigned i;
    for (i = 0; i < 8; i++) { send_bit((b & 0x80u) ? 1u : 0u); b = (unsigned char)(b << 1); }
    return recv_bit();
}

/* 收一字节; ack=1 回 ACK (中间字节), ack=0 回 NACK (最后一字节) */
static unsigned char recv_byte(unsigned ack)
{
    unsigned char v = 0;
    unsigned i;
    for (i = 0; i < 8; i++) v = (unsigned char)((v << 1) | (recv_bit() & 1u));
    send_bit(ack ? 0u : 1u);
    return v;
}

/* 写"设备地址 + 寄存器地址": 不发 STOP (留给读的时候发重复起始) */
static int MLX_WriteCmd(unsigned char devAddr, unsigned char cmd)
{
    i2c_start();
    if (send_byte((unsigned char)(devAddr << 1))) { i2c_stop(); return -5; }  /* 地址没应答 */
    if (send_byte(cmd))                           { i2c_stop(); return -5; }  /* 命令没应答 */
    return 0;
}

/* 重复起始 + 读 len 字节 (前 n-1 字节回 ACK, 最后一字节回 NACK) + STOP */
static int MLX_ReadBytes(unsigned char devAddr, unsigned char *buf, uint8_t len)
{
    uint8_t i;

    i2c_start();
    if (send_byte((unsigned char)((devAddr << 1) | 0x01))) { i2c_stop(); return -5; }
    for (i = 0; i < len; i++) buf[i] = recv_byte((i + 1u < len) ? 1u : 0u);
    i2c_stop();
    return 0;
}

/*---------------------------------------------------------------------------
 * 初始化: 把 PB17/PB18 配成 GPIO 并置空闲 (上电调一次)
 *-------------------------------------------------------------------------*/
void MLX90614_Init(void)
{
    DL_GPIO_initDigitalOutput(MLX_SCL_PINCM);
    DL_GPIO_enableOutput(MLX_SCL_PORT, MLX_SCL_PIN);
    scl_h();                     /* SCL 拉高 */
    sda_release();               /* SDA 放开 -> 上拉拉高 */
    delay_us(10);
}

/*---------------------------------------------------------------------------
 * 诊断: 探测一个地址上有没有器件 (判据 = 有没有 ACK / 有没有读回字节)
 *-------------------------------------------------------------------------*/
int MLX90614_ProbeAddr(unsigned char addr7)
{
    unsigned char rx[2];

    if (MLX_WriteCmd(addr7, MLX90614_REG_TOBJ1) != 0) return -1;
    delay_us(200);
    if (MLX_ReadBytes(addr7, rx, 2) != 0) return -2;
    return 0;
}

/*---------------------------------------------------------------------------
 * 总线扫描: 找出 0x00~0x77 里所有"能读回数据"的地址 (只在 PB17/PB18 这条线上扫)
 *-------------------------------------------------------------------------*/
int MLX90614_BusScan(unsigned char *list, int maxn)
{
    unsigned char a;
    int n = 0;

    for (a = 0x00; a <= 0x77; a++)
    {
        if (MLX90614_ProbeAddr(a) != 0) continue;
        if (list && (n < maxn)) list[n] = a;
        n++;
    }
    return n;
}

/*---------------------------------------------------------------------------
 * 标准 SMBus CRC-8: poly=0x07, init=0x00, MSB-first, 无反转 / 无末异或
 *-------------------------------------------------------------------------*/
unsigned char MLX90614_CRC8(const unsigned char *data, unsigned char len)
{
    unsigned char crc = 0x00;
    unsigned char i;

    while (len--)
    {
        crc ^= *data++;
        for (i = 0; i < 8; i++)
            crc = (unsigned char)((crc & 0x80) ? ((crc << 1) ^ 0x07u) : (crc << 1));
    }
    return crc;
}

/*---------------------------------------------------------------------------
 * 读一个 16 位字: 写命令 -> 重复起始读 DataL/DataH/PEC -> 校验 -> 不符重试
 *-------------------------------------------------------------------------*/
int MLX90614_ReadWord(unsigned char SlaveAddr, unsigned char RegAddr, uint16_t *raw)
{
    unsigned char rx[3];
#if MLX90614_CHECK_PEC
    unsigned char msg[5];
#endif
    uint16_t last = 0;
    int      got  = 0;
    int      attempt;

    for (attempt = 0; attempt < MLX90614_RETRY; attempt++)
    {
        if (MLX_WriteCmd(SlaveAddr, RegAddr) != 0) continue;
        if (MLX_ReadBytes(SlaveAddr, rx, 3)  != 0) continue;

        last = (uint16_t)(((uint16_t)rx[1] << 8) | rx[0]);
        got  = 1;

#if MLX90614_CHECK_PEC
        msg[0] = (unsigned char)(SlaveAddr << 1);
        msg[1] = RegAddr;
        msg[2] = (unsigned char)((SlaveAddr << 1) | 0x01);
        msg[3] = rx[0];
        msg[4] = rx[1];
        if (MLX90614_CRC8(msg, 5) == rx[2])
        {
            if (raw) *raw = last;
            return 0;
        }
#else
        if (raw) *raw = last;
        return 0;
#endif
    }

    if (got)
    {
        mlx_pec_err++;
        if (raw) *raw = last;
        return 1;
    }
    return -1;
}

/*---------------------------------------------------------------------------
 * 上电自动探测地址: 0x5A (手册默认) -> 0x00 (例程里那两个版本用的)
 *-------------------------------------------------------------------------*/
void MLX90614_AutoDetect(void)
{
    uint16_t raw;

    mlx_addr = MLX90614_I2C_ADDR;
    if (MLX90614_ReadWord(mlx_addr, MLX90614_REG_TOBJ1, &raw) >= 0) return;

    mlx_addr = 0x00u;
    if (MLX90614_ReadWord(mlx_addr, MLX90614_REG_TOBJ1, &raw) >= 0) return;

    mlx_addr = MLX90614_I2C_ADDR;      /* 都不行 -> 回到默认 */
}

/*---------------------------------------------------------------------------
 * 温度: ℃ = raw * 0.02 - 273.15
 *-------------------------------------------------------------------------*/
float MLX90614_ReadTemp(void)
{
    uint16_t raw;

    if (MLX90614_ReadWord(mlx_addr, MLX90614_REG_TOBJ1, &raw) < 0)
        return MLX90614_ERR;
    return (float)raw * 0.02f - 273.15f;
}

/* 兼容旧接口: 读寄存器并转 ℃; 失败返回 0.0f */
float MLX90614_Read(unsigned char SlaveAddr, unsigned char RegAddr)
{
    uint16_t raw;

    if (MLX90614_ReadWord(SlaveAddr, RegAddr, &raw) < 0) return 0.0f;
    return (float)raw * 0.02f - 273.15f;
}

uint32_t      MLX90614_GetPecErrCount(void) { return mlx_pec_err; }
unsigned char MLX90614_GetAddr(void)        { return mlx_addr; }
