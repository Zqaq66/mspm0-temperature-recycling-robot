/**
 * @file  imu_uart_driver.c
 * @brief YbIMU 六轴陀螺仪 UART 驱动实现
 *
 * 引脚依赖: 全部通过 imu_uart_driver.h → ti_msp_dl_config.h (SysConfig)
 * 在 SysConfig 中修改 IMU 引脚后无需手动修改本文件。
 *
 * 模块组成:
 *   1. 环形缓冲区 (256 字节)
 *   2. 数据转换辅助函数
 *   3. 传感器数据内部缓存
 *   4. 帧解析器 (按功能码分发)
 *   5. 状态机处理器 (从环形缓冲消费字节)
 *   6. 命令帧发送 (阻塞式轮询 TX)
 *   7. UART0 硬件初始化 (GPIO 由 SysConfig 管理, 本函数仅配 UART 寄存器)
 *   8. UART0 RX 中断服务例程
 *   9. 公共数据读取 API
 *  10. 校准 API
 */

#include "imu_uart_driver.h"
#include "Time.h"

#include <string.h>   /* memcpy */

/*===========================================================================
 * 1. 环形缓冲区 / Ring Buffer
 *===========================================================================*/

static volatile uint8_t  s_rx_buffer[IMU_UART_RX_BUF_SIZE];
static volatile uint16_t s_rx_write_index = 0;
static volatile uint16_t s_rx_read_index  = 0;

static inline uint16_t _rxbuf_next(uint16_t index)
{
    return (uint16_t)((index + 1u) % IMU_UART_RX_BUF_SIZE);
}

static inline int _rxbuf_is_empty(void)
{
    return (s_rx_write_index == s_rx_read_index);
}

static inline void _rxbuf_push(uint8_t byte_value)
{
    uint16_t next_index = _rxbuf_next(s_rx_write_index);
    if (next_index == s_rx_read_index) {
        /* 缓冲区满时丢弃最旧字节 / drop oldest byte when full */
        s_rx_read_index = _rxbuf_next(s_rx_read_index);
    }
    s_rx_buffer[s_rx_write_index] = byte_value;
    s_rx_write_index = next_index;
}

static inline int _rxbuf_pop(uint8_t *out_byte)
{
    if (_rxbuf_is_empty()) {
        return -1;
    }
    *out_byte = s_rx_buffer[s_rx_read_index];
    s_rx_read_index = _rxbuf_next(s_rx_read_index);
    return 0;
}

/*===========================================================================
 * 2. 数据转换辅助函数 / Data Conversion Helpers
 *===========================================================================*/

/** 小端 2 字节 → int16 (little-endian) */
static int16_t to_int16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[1] << 8) | (uint16_t)bytes[0]);
}

/** 小端 4 字节 → float (IEEE 754, little-endian) */
static float to_float(const uint8_t *bytes)
{
    float v;
    memcpy(&v, bytes, sizeof(float));
    return v;
}

/** 累加校验和 (所有字节求和取低 8 位) */
static uint8_t calculate_checksum(const uint8_t *data, uint16_t len)
{
    uint32_t s = 0;
    for (uint16_t i = 0; i < len; ++i) {
        s += data[i];
    }
    return (uint8_t)(s & 0xFFU);
}

/*===========================================================================
 * 3. 传感器数据内部缓存 / Internal Cached Sensor State
 *===========================================================================*/

static volatile float s_ax = 0.0f, s_ay = 0.0f, s_az = 0.0f;
static volatile float s_gx = 0.0f, s_gy = 0.0f, s_gz = 0.0f;
static volatile float s_mx = 0.0f, s_my = 0.0f, s_mz = 0.0f;
static volatile float s_roll = 0.0f, s_pitch = 0.0f, s_yaw = 0.0f;
static volatile float s_q0 = 0.0f, s_q1 = 0.0f, s_q2 = 0.0f, s_q3 = 0.0f;
static volatile float s_height = 0.0f, s_temperature = 0.0f, s_pressure = 0.0f, s_pressure_contrast = 0.0f;
static volatile int   s_version_high = -1, s_version_mid = 0, s_version_low = 0;
static volatile uint8_t s_last_rx_function = 0;
static volatile int16_t s_last_rx_state = 0;

/*===========================================================================
 * 4. 帧数据解析 / Parse One Complete Frame
 *===========================================================================*/

static void _parse_frame_data(uint8_t frame_function, const uint8_t *frame_data)
{
    if (frame_function == IMU_FUNC_RAW_ACCEL) {
        /* 加速度: int16 → g (量程 ±16g) */
        float accel_ratio = 16.0f / 32767.0f;
        s_ax = to_int16(&frame_data[0])  * accel_ratio;
        s_ay = to_int16(&frame_data[2])  * accel_ratio;
        s_az = to_int16(&frame_data[4])  * accel_ratio;

        /* 陀螺仪: int16 → rad/s (量程 ±2000°/s) */
        float deg_to_rad = 3.14159265358979323846f / 180.0f;
        float gyro_ratio  = (2000.0f / 32767.0f) * deg_to_rad;
        s_gx = to_int16(&frame_data[6])  * gyro_ratio;
        s_gy = to_int16(&frame_data[8])  * gyro_ratio;
        s_gz = to_int16(&frame_data[10]) * gyro_ratio;

        /* 磁力计: int16 → uT (量程 ±800 uT) */
        float mag_ratio = 800.0f / 32767.0f;
        s_mx = to_int16(&frame_data[12]) * mag_ratio;
        s_my = to_int16(&frame_data[14]) * mag_ratio;
        s_mz = to_int16(&frame_data[16]) * mag_ratio;

    } else if (frame_function == IMU_FUNC_EULER) {
        /* 欧拉角: float (弧度) */
        s_roll  = to_float(&frame_data[0]);
        s_pitch = to_float(&frame_data[4]);
        s_yaw   = to_float(&frame_data[8]);

    } else if (frame_function == IMU_FUNC_QUAT) {
        /* 四元数: float w,x,y,z */
        s_q0 = to_float(&frame_data[0]);
        s_q1 = to_float(&frame_data[4]);
        s_q2 = to_float(&frame_data[8]);
        s_q3 = to_float(&frame_data[12]);

    } else if (frame_function == IMU_FUNC_BARO) {
        /* 气压计: float */
        s_height            = to_float(&frame_data[0]);
        s_temperature       = to_float(&frame_data[4]);
        s_pressure          = to_float(&frame_data[8]);
        s_pressure_contrast = to_float(&frame_data[12]);

    } else if (frame_function == IMU_FUNC_VERSION) {
        s_version_high = (int)frame_data[0];
        s_version_mid  = (int)frame_data[1];
        s_version_low  = (int)frame_data[2];

    } else if (frame_function == IMU_FUNC_RETURN_STATE) {
        s_last_rx_function = frame_data[0];
        s_last_rx_state    = (int16_t)frame_data[1];
    }
}

/*===========================================================================
 * 5. 环形缓冲消费 + 帧解析状态机 / Ring Buffer Consumer + Frame Parser FSM
 *===========================================================================*/

/**
 * @brief 消费环形缓冲中的字节，按协议状态机提取完整帧
 * @note  应在主循环或调度器任务中周期性调用
 */
void IMU_Process(void)
{
    enum {
        RX_STATE_EXPECT_HEAD1 = 0,
        RX_STATE_EXPECT_HEAD2,
        RX_STATE_EXPECT_LENGTH,
        RX_STATE_EXPECT_FUNCTION,
        RX_STATE_COLLECT_DATA
    };

    static uint8_t  rx_state       = RX_STATE_EXPECT_HEAD1;
    static uint8_t  frame_length   = 0;
    static uint8_t  frame_function = 0;
    static uint8_t  frame_buffer[64]; /* 数据区 + 校验和 */
    static uint16_t frame_index    = 0;

    uint8_t current_byte = 0;

    while (_rxbuf_pop(&current_byte) == 0) {
        switch (rx_state) {

        case RX_STATE_EXPECT_HEAD1:
            rx_state = (current_byte == FRAME_HEAD1)
                       ? RX_STATE_EXPECT_HEAD2
                       : RX_STATE_EXPECT_HEAD1;
            break;

        case RX_STATE_EXPECT_HEAD2:
            rx_state = (current_byte == FRAME_HEAD2)
                       ? RX_STATE_EXPECT_LENGTH
                       : RX_STATE_EXPECT_HEAD1;
            break;

        case RX_STATE_EXPECT_LENGTH:
            frame_length = current_byte;
            rx_state = RX_STATE_EXPECT_FUNCTION;
            break;

        case RX_STATE_EXPECT_FUNCTION:
            frame_function = current_byte;
            frame_index = 0;
            rx_state = RX_STATE_COLLECT_DATA;
            break;

        case RX_STATE_COLLECT_DATA: {
            /* data_length = 帧总长 - 4 (head1+head2+len+func) */
            uint16_t data_length = (frame_length >= 4)
                                   ? (uint16_t)(frame_length - 4)
                                   : 0;

            if (data_length == 0 || data_length > sizeof(frame_buffer)) {
                /* 非法长度 → 回到头字节搜索 */
                rx_state = RX_STATE_EXPECT_HEAD1;
                break;
            }

            frame_buffer[frame_index++] = current_byte;

            if (frame_index >= data_length) {
                /* 数据区收齐 → 验证校验和 */
                uint8_t calculated = (uint8_t)(
                    FRAME_HEAD1 + FRAME_HEAD2 + frame_length + frame_function);
                for (uint16_t i = 0; i < data_length - 1; ++i) {
                    calculated = (uint8_t)(calculated + frame_buffer[i]);
                }

                uint8_t received = frame_buffer[data_length - 1];
                if (calculated == received) {
                    _parse_frame_data(frame_function, frame_buffer);
                }
                /* 无论校验是否通过都回到头字节搜索 */
                rx_state = RX_STATE_EXPECT_HEAD1;
            }
        } break;

        default:
            rx_state = RX_STATE_EXPECT_HEAD1;
            break;
        }
    }
}

/*===========================================================================
 * 6. 命令帧发送 / Command Frame Sender (blocking polled TX)
 *===========================================================================*/

/** 发送单字节 (阻塞轮询) */
static void _imu_send_byte(uint8_t data)
{
    while (DL_UART_isBusy(IMU_UART_INST) == true) {
        /* wait until TX shift register is free */
    }
    DL_UART_Main_transmitData(IMU_UART_INST, data);
}

/** 发送字节数组 (阻塞轮询) */
static void _imu_send_array(const uint8_t *pData, uint8_t length)
{
    while (length--) {
        _imu_send_byte(*pData);
        pData++;
    }
}

/**
 * @brief 构建并发送命令帧
 * @param function  功能码
 * @param params    参数数组
 * @param param_len 参数长度 (0~3)
 * @return 0=成功, -1=参数错误
 */
int IMU_SendCommand(uint8_t function, const uint8_t *params, uint8_t param_len)
{
    if (param_len > 3 || (param_len > 0 && params == NULL)) {
        return -1;
    }

    /* 帧缓冲: HEAD1(1) + HEAD2(1) + LEN(1) + FUNC(1) + PARAMS(≤3) + CHK(1) = ≤8 */
    uint8_t frame[8] = {
        FRAME_HEAD1,
        FRAME_HEAD2,
        0,          /* length — 后续计算 */
        function,
        0, 0, 0,    /* params 占位 */
        0           /* checksum 占位 */
    };

    for (uint8_t i = 0; i < param_len; ++i) {
        frame[4 + i] = params[i];
    }

    /* 帧总长 = 4 (固定头) + param_len + 1 (校验) */
    uint8_t frame_len = (uint8_t)(4 + param_len + 1);
    frame[2] = frame_len;

    /* 计算校验和 (帧长 - 1 个字节累加) */
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < frame_len - 1; ++i) {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    frame[frame_len - 1] = checksum;

    _imu_send_array(frame, frame_len);
    return 0;
}

/*===========================================================================
 * 7. UART0 硬件初始化 / Manual UART0 Hardware Initialization
 *===========================================================================*/

/**
 * @brief IMU UART0 硬件初始化
 *
 * GPIO 引脚复用由 SysConfig (SYSCFG_DL_GPIO_init) 管理,
 * 本函数仅执行 UART 寄存器级初始化 (复位/时钟/波特率/中断/NVIC)。
 *
 * 调用前提: SYSCFG_DL_init() 已执行 (GPIO + 时钟 + 电源已初始化)
 */
void IMU_Init(void)
{
    DL_GPIO_enableOutput(IMU_UART_TX_PORT, IMU_UART_TX_PIN);

    /* RX 引脚加内部上拉 (直接 RMW, 保留 SysConfig 的外设功能配置) */
    IOMUX->SECCFG.PINCM[IMU_UART_RX_IOMUX] |= (uint32_t)DL_GPIO_RESISTOR_PULL_UP;

    DL_GPIO_disableOutput(IMU_UART_RX_PORT, IMU_UART_RX_PIN);
    DL_UART_Main_disableInterrupt(IMU_UART_INST,
                                   DL_UART_MAIN_INTERRUPT_TX);

    NVIC_ClearPendingIRQ(IMU_UART_INT_IRQN);
    NVIC_SetPriority(IMU_UART_INT_IRQN, IMU_UART_IRQ_PRIORITY);
    NVIC_EnableIRQ(IMU_UART_INT_IRQN);
}

/*===========================================================================
 * 8. UART0 RX 中断服务例程 / ISR
 *===========================================================================*/

/**
 * @brief UART0 RX 中断处理
 *
 * 从 UART RX FIFO 读取字节，批量写入环形缓冲区。
 * 使用 128 字节本地缓冲减少 ISR 内对共享环形缓冲的写入次数。
 */
static volatile uint32_t s_rx_byte_count = 0;  /* ISR 接收总字节数 (调试用) */
static volatile uint8_t  s_raw_dump[64];       /* 最近 64 字节原始快照 */
static volatile uint8_t  s_raw_dump_idx = 0;

void IMU_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(IMU_UART_INST)) {

    case DL_UART_IIDX_RX: {
        uint8_t byte_val = DL_UART_Main_receiveData(IMU_UART_INST);
        s_rx_byte_count++;
        s_raw_dump[s_raw_dump_idx] = byte_val;
        s_raw_dump_idx = (uint8_t)((s_raw_dump_idx + 1) % 64);

        /* 直接推入环形缓冲, 不经本地批量缓存
         * 避免低波特率下批量缓冲填满过慢导致数据积压 */
        _rxbuf_push(byte_val);
        break;
    }

    default:
        break;
    }
}

/*===========================================================================
 * 9. 公共数据读取 API / Public Data Read Functions
 *===========================================================================*/

int IMU_GetAccelerometer(float out[3])
{
    if (!out) return -1;
    out[0] = s_ax; out[1] = s_ay; out[2] = s_az;
    return 0;
}

int IMU_GetGyroscope(float out[3])
{
    if (!out) return -1;
    out[0] = s_gx; out[1] = s_gy; out[2] = s_gz;
    return 0;
}

int IMU_GetMagnetometer(float out[3])
{
    if (!out) return -1;
    out[0] = s_mx; out[1] = s_my; out[2] = s_mz;
    return 0;
}

int IMU_GetQuaternion(float out[4])
{
    if (!out) return -1;
    out[0] = s_q0; out[1] = s_q1; out[2] = s_q2; out[3] = s_q3;
    return 0;
}

int IMU_GetEuler(float out[3])
{
    if (!out) return -1;
    /* 传感器上报的是弧度，转换为角度 */
    static const float RAD2DEG = 57.2957795f;
    out[0] = s_roll  * RAD2DEG;
    out[1] = s_pitch * RAD2DEG;
    out[2] = s_yaw   * RAD2DEG;
    return 0;
}

int IMU_GetBarometer(float out[4])
{
    if (!out) return -1;
    out[0] = s_height;
    out[1] = s_temperature;
    out[2] = s_pressure;
    out[3] = s_pressure_contrast;
    return 0;
}

int IMU_GetAll(imu_measurement_t *out)
{
    if (!out) return -1;
    IMU_GetAccelerometer(out->accel);
    IMU_GetGyroscope(out->gyro);
    IMU_GetMagnetometer(out->mag);
    IMU_GetQuaternion(out->quat);
    IMU_GetEuler(out->euler);
    IMU_GetBarometer(out->baro);
    return 0;
}

/** @brief 调试: 获取 ISR 已接收的原始字节总数 */
uint32_t IMU_GetRxByteCount(void)
{
    return s_rx_byte_count;
}

/**
 * @brief 调试: 抓取最近收到的原始字节快照 (环形, 最多 64 字节)
 * @param buf     用户提供的缓冲区
 * @param max_len 期望拷贝的字节数
 * @return 实际拷贝的字节数
 */
uint16_t IMU_DumpRawBytes(uint8_t *buf, uint16_t max_len)
{
    if (!buf || max_len == 0) return 0;

    uint32_t total = s_rx_byte_count;
    uint8_t  count;
    uint8_t  valid = (uint8_t)((total < 64U) ? (uint8_t)total : 64U);

    count = (uint8_t)((valid < max_len) ? valid : (uint8_t)max_len);
    if (count == 0) return 0;

    /*
     * 缓冲未绕回 (total < 64): 有效数据在 [0, s_raw_dump_idx)
     * 缓冲已绕回 (total >= 64): 最旧数据在 s_raw_dump_idx, 读取 64 个
     */
    uint8_t start = (total < 64U) ? 0 : s_raw_dump_idx;

    for (uint8_t i = 0; i < count; ++i) {
        buf[i] = s_raw_dump[(start + i) % 64];
    }
    return count;
}

void IMU_ClearData(void)
{
    s_ax = s_ay = s_az = 0.0f;
    s_gx = s_gy = s_gz = 0.0f;
    s_mx = s_my = s_mz = 0.0f;
    s_roll = s_pitch = s_yaw = 0.0f;
    s_q0 = s_q1 = s_q2 = s_q3 = 0.0f;
    s_height = s_temperature = s_pressure = s_pressure_contrast = 0.0f;
}

/*===========================================================================
 * 10. 校准 API / Calibration Functions
 *===========================================================================*/

/**
 * @brief 等待校准完成 (内部辅助函数)
 * @param function   触发校准的功能码
 * @param timeout_ms 超时时间 (ms), 0 = 永不超时
 * @return 1=成功, 0=失败, -1=超时
 */
static int _calibration_wait(uint8_t function, uint32_t timeout_ms)
{
    uint32_t elapsed = 0;

    while (1) {
        /* 持续消费环形缓冲，确保校准应答帧被解析 */
        IMU_Process();

        if (s_last_rx_function == function) {
            return (int)s_last_rx_state;
        }

        if (timeout_ms != 0 && elapsed >= timeout_ms) {
            return -1;
        }

        delay_ms(1);
        if (timeout_ms != 0) {
            ++elapsed;
        }
    }
}

int IMU_CalibrateImu(void)
{
    s_last_rx_function = 0;
    s_last_rx_state = -1;

    uint8_t payload[2] = {0x01, 0x5F};
    int rc = IMU_SendCommand(IMU_FUNC_CALIB_IMU, payload,
                              (uint8_t)sizeof(payload));
    if (rc != 0) {
        return rc;
    }

    return _calibration_wait(IMU_FUNC_CALIB_IMU, 7000);
}

int IMU_CalibrateMag(void)
{
    s_last_rx_function = 0;
    s_last_rx_state = -1;

    uint8_t payload[2] = {0x01, 0x5F};
    int rc = IMU_SendCommand(IMU_FUNC_CALIB_MAG, payload,
                              (uint8_t)sizeof(payload));
    if (rc != 0) {
        return rc;
    }

    return _calibration_wait(IMU_FUNC_CALIB_MAG, 0);
}

int IMU_CalibrateTemp(float now_temperature)
{
    if (now_temperature > 50.0f || now_temperature < -50.0f) {
        return -1;
    }

    s_last_rx_function = 0;
    s_last_rx_state = -1;

    int16_t temp_raw = (int16_t)(now_temperature * 100.0f);
    uint8_t payload[3] = {
        (uint8_t)(temp_raw & 0xFF),
        (uint8_t)((temp_raw >> 8) & 0xFF),
        0x5F
    };

    int rc = IMU_SendCommand(IMU_FUNC_CALIB_TEMP, payload,
                              (uint8_t)sizeof(payload));
    if (rc != 0) {
        return rc;
    }

    return _calibration_wait(IMU_FUNC_CALIB_TEMP, 2000);
}

int IMU_ResetUserData(void)
{
    uint8_t payload[2] = {0x01, 0x5F};
    return IMU_SendCommand(IMU_FUNC_RESET_FLASH, payload,
                           (uint8_t)sizeof(payload));
}
