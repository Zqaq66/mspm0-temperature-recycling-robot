# DZZH 移动测温回收机器人 —— PCB 接线汇总与注意事项

> 权威来源（按当前工作区实测）：`ti_msp_dl_config.h` + `empty.syscfg`（SysConfig 生成）+ `app/MLX90614.c/.h` + `app/TW.c/.h` + `empty.c`。
> 与任何注释/旧文档冲突时，**以 `ti_msp_dl_config.h` 和对应驱动 `.c` 为准**。
> 更新：2026-10-14 —— 反映【MLX90614 改为 PB17/PB18 软件 I2C】+【语音改为 ASRPRO2.0 单线 UART0】；
> 并确认本板形态 = **天猛星 MSPM0G3507 开发板（插上去）+ 全部用现成模块** 的载板。
>
> **配套交付物（可直接拿去画板）**：`pcb/`
> `connection_matrix.csv`（逐脚矩阵）、`netlist.txt`（61 网）、`BOM.csv`、
> `validation_report.txt`（对固件 62 项校验）、`carrier_block_diagram.png`、
> **`DZZH_carrier.epro`（立创EDA专业版工程，推荐直接导入）**、
> `DZZH_carrier.kicad_sch`（KiCad 8）、`DZZH_carrier_sch_preview.png`、`design_guide.md`。
> 工程含 **43 个器件（含额外按键 K1/K2/K3）**、**16 个真实封装**、64 个网络。

---

## 0. 一句话总览

- **板形态**：**载板** —— 插一块 **天猛星 MSPM0G3507** 开发板（MCU 最小系统 / 电源 / 时钟 / 调试都由它提供），
  外围（TB6612、灰度、MLX90614、IMU、LCD、LORA、授时、语音）全部用**现成模块**；本板只做转接 + 电源 + 电机驱动。
- **MCU**：MSPM0G3507，封装 **LQFP-64 (PM)**；CPUCLK = BUSCLK = **80 MHz**（内部 SYSOSC + SYSPLL0）→ **不需要外部晶振**。
- **必须 3.3 V 逻辑**：TB6612 的 VCC、所有 I2C/SPI/UART 模块都用 3.3 V；**12 V 只进 TB6612 的 VM**。
- **两条 I2C 互不相干**：
  - **硬件 I2C0（PA0/PA1）只挂循迹灰度**（芯流 AuroraT8，0x40）。
  - **MLX90614 走【软件模拟 I2C】：SCL=PB17 / SDA=PB18**（独立空脚，和灰度总线分开，可同时接）。
  - **硬件 I2C1（PA29/PA30）挂授时** ESP32-S2（0x42）。
- **串口**：UART0=语音 ASRPRO2.0（PA10）、UART1=IMU（PA8/PA9）、UART2=空闲（PB15/16）、**UART3=LORA（PB2/PB3）= `printf` 出口**。全部按 3.3 V 电平、交叉 + 共地。
- **SPI1** 驱动 ST7735S LCD（PB8/PB9 + 4 根 GPIO）。
- **ADC0/PA27** 测 12 V 电池（板上 1/4 分压）。

---

## 1. 电源树

| 电源 | 供到 | 备注 |
|---|---|---|
| 12 V 电池 + | TB6612 #1/#2 的 `VM`（两片并联） | 每片 VM 就近放 100 µF 电解 + 100 nF 瓷片 |
| 12 V 电池 − | 全部 GND（**必须共地**） | 电机电流地 / 逻辑地 单点汇接 |
| 3.3 V（载板） | 两片 TB6612 `VCC`、灰度、MLX90614、LCD、IMU、LORA、编码器 | 载板 12V→3.3V 降压提供；**不接天猛星 3V3（那是板载输出，会倒灌）** |
| 3.3 V（单独一路） | 授时 ESP32-S2 模块 | 独立 LDO / 小降压（峰值 300~500 mA），别从载板 3V3 挤 |
| 5 V（载板） | 语音 ASRPRO2.0 + 经 D2(SS34) 给天猛星 5V | ASRPRO >500 mA、外接喇叭；D2 防倒灌 |
| 电池分压点 | ADC0/PA27 | 1/4 分压（12.6 V→3.15 V），节点加 100 nF 到地 |

> TB6612 门限：`VIH = 0.7 × VCC`。VCC 接 **3.3 V** 时门限 2.31 V（3.3 V 逻辑才稳）；VCC 接 5 V 则门限 3.5 V > 3.3 V，识别会不稳。
>
> ⚠️ 天猛星只从 **5V 脚**取电（载板 +5V → D2 → 5V）；它的 **3V3 脚是板载稳压输出**，悬空/只做测试点，**不要并到载板 3V3**。

---

## 2. MCU 引脚总表（LQFP-64 PM）

| 引脚 | PINCM | 封装脚 | 功能 | 接到哪 |
|---|---|---|---|---|
| PA0 | 1 | — | I2C0_SDA | 循迹灰度 |
| PA1 | 2 | — | I2C0_SCL | 循迹灰度 |
| PA7 | 14 | — | TIMG7_CCP1 | TB6612#2 `PWMB` → **D 左后** |
| PA8 | 19 | — | UART1_TX | IMU `RX` |
| PA9 | 20 | — | UART1_RX | IMU `TX` |
| PA10 | 21 | — | UART0_TX | 语音 ASRPRO 串口1 `RX` |
| PA11 | 22 | — | UART0_RX | （空，只发不接） |
| PA12 | 34 | — | TIMG0_CCP0 | TB6612#1 `PWMA` → **A 左前** |
| PA13 | 35 | — | TIMG0_CCP1 | TB6612#1 `PWMB` → **B 右前** |
| PA14 | 36 | 7 | GPIO 输入(中断) | 编码器 **D 左后 A 相** |
| PA15 | 37 | 8 | GPIO 输入(中断) | 编码器 **D 左后 B 相** |
| PA16 | 38 | 9 | GPIO 输入 | HC05 蓝牙 STATE（未用） |
| PA17 | 39 | — | TIMG7_CCP0 | TB6612#2 `PWMA` → **C 右后** |
| PA18 | — | — | （空） | S1/BSL：**复位时为高会进 BSL**，保留低或接 BSL 按键 |
| PA19 | — | — | SWDIO | SWD 调试口 |
| PA20 | — | — | SWCLK | SWD 调试口 |
| PA21 | 46 | 17 | GPIO 输入(中断) | 编码器 **A 左前 A 相** |
| PA24 | 54 | 25 | GPIO 输入(中断) | 编码器 **C 右后 A 相** |
| PA25 | 55 | 26 | GPIO 输入(中断) | 编码器 **C 右后 B 相** |
| PA26 | 59 | 30 | GPIO 输入(中断) | 编码器 **A 左前 B 相** |
| PA27 | — | — | ADC0 CH0 | 电池分压检测 |
| PA28 | 3 | 35 | GPIO 输入(中断) | 编码器 **B 右前 B 相** |
| PA29 | 4 | — | I2C1_SCL | 授时模块 SCL |
| PA30 | 5 | — | I2C1_SDA | 授时模块 SDA |
| PA31 | 6 | 39 | GPIO 输入(中断) | 编码器 **B 右前 A 相** |
| PB0 | 12 | 47 | GPIO 输出 | 两片 TB6612 `STBY`（高使能） |
| PB2 | 15 | — | UART3_TX | LORA `RXD` |
| PB3 | 16 | — | UART3_RX | LORA `TXD` |
| PB4 | 17 | 52 | GPIO 输出 | TB6612#1 `BIN1` |
| PB5 | 18 | 53 | GPIO 输出 | TB6612#1 `BIN2` |
| PB6 | 23 | 58 | GPIO 输出 | TB6612#2 `AIN1` |
| PB7 | 24 | 59 | GPIO 输出 | TB6612#2 `AIN2` |
| PB8 | 25 | — | SPI1 PICO | LCD `SDA/MOSI` |
| PB9 | 26 | — | SPI1 SCLK | LCD `SCLK` |
| PB10 | 27 | 62 | GPIO 输出 | LCD `RES` |
| PB11 | 28 | 63 | GPIO 输出 | LCD `DC` |
| PB12 | 29 | 64 | GPIO 输出 | TB6612#2 `BIN1` |
| PB13 | 30 | 1 | GPIO 输出 | TB6612#2 `BIN2` |
| PB14 | 31 | 2 | GPIO 输出 | LCD `CS` |
| PB15 | 32 | — | UART2_TX | 空闲 |
| PB16 | 33 | — | UART2_RX | 空闲 |
| **PB17** | 43 | 14 | **软件 I2C — SCL** | **MLX90614 `SCL`** |
| **PB18** | 44 | 15 | **软件 I2C — SDA** | **MLX90614 `SDA`** |
| PB19 | 45 | 16 | GPIO 输出 | TB6612#1 `AIN1` |
| PB20 | 48 | 19 | GPIO 输出 | TB6612#1 `AIN2` |
| PB21 | 49 | 20 | GPIO 输入(内部上拉) | S2 按键，另一端接 GND |
| PB26 | 57 | 28 | GPIO 输出 | LCD `BLK` 背光 |

> 封装脚号取自 `ti_msp_dl_config.h` 里 SysConfig 标注的 `package pin`。
> ⚠️ PB17/PB18 在 SysConfig 里仍叫 `TW_TW_0/TW_TW_1`（历史命名），**现在实际是 MLX90614 的软 I2C**（`MLX90614_Init()` 里会重新配成 GPIO）。

---

## 3. 分组接线

### 3.1 电机驱动 TB6612 ×2

PWM **20 kHz**（TIMG0/TIMG7，`clockDivider=4` → 20 MHz 计数 → 20 kHz）；SysConfig 里两路 PWM 都 `invert=true`（固件有意，照常接）。

| 轮位 | PWM | 方向 1 | 方向 2 | 驱动片通道 |
|---|---|---|---|---|
| **A 左前** | PA12 (`PWMA`) | PB19 (`AIN1`) | PB20 (`AIN2`) | TB6612#1 A 通道 |
| **B 右前** | PA13 (`PWMB`) | PB4 (`BIN1`) | PB5 (`BIN2`) | TB6612#1 B 通道 |
| **C 右后** | PA17 (`PWMA`) | PB6 (`AIN1`) | PB7 (`AIN2`) | TB6612#2 A 通道 |
| **D 左后** | PA7 (`PWMB`) | PB12 (`BIN1`) | PB13 (`BIN2`) | TB6612#2 B 通道 |

- **`STBY`（两片并联）→ PB0**，高使能。建议 PB0 到 GND 加 10 k 下拉：MCU 复位期间 GPIO 高阻 → STBY 拉低 → 电机自动待机，防上电乱动。
- 输出 `AO1/AO2`、`BO1/BO2` 接电机 `M+/M−`。
- ⚠️ **芯片丝印的 A/B 通道 ≠ 轮位 A/B**：#1 片 A 通道驱动 A 轮、B 通道驱动 B 轮，别按丝印对号。
- 方向反了**不要拆线**：改 `app/Motor.c` 的 `MOTOR_x_REVERSE`。

### 3.2 编码器 ×4（差分 A/B 相）

| 轮位 | A 相 | B 相 |
|---|---|---|
| A 左前 | PA21 | PA26 |
| B 右前 | PA31 | PA28 |
| C 右后 | PA24 | PA25 |
| D 左后 | PA14 | PA15 |

- 这 8 脚 SysConfig 里是 **无内部上下拉（RESISTOR_NONE）**。编码器若为**开漏输出**，每相各加 **10 k 上拉到 3.3 V**。
- 编码器 VCC → 3.3 V，GND → 共地。
- 相序反了改 `app/Encoder.c` 的 `ENC_x_REVERSE`。
- ⚠️ **循迹/转向不用编码器**（麦轮打滑会让速度环失真）——按普通 IO 走线即可。

### 3.3 循迹灰度（硬件 I2C0，独占）

| 信号 | 引脚 |
|---|---|
| SDA | PA0 |
| SCL | PA1 |

- 400 kHz Fast，7 位地址 **0x40**，读寄存器 0x09 八路模拟量。
- **必须外部上拉**（SysConfig 未开内部上拉）：**2.2 k ~ 4.7 k 到 3.3 V**。走线 ≤10 cm。
- 探头朝下、8 路垂直前进方向横排、居中装。

### 3.4 红外测温 MLX90614（软件模拟 I2C，PB17/PB18）

| 信号 | 引脚 | 说明 |
|---|---|---|
| `SCL` | **PB17** | MCU 推挽输出（总线只有主机驱动 SCL） |
| `SDA` | **PB18** | MCU 开漏（输出低 / 放开靠上拉），固件用内部 ~40 k 上拉 |
| `VIN` | 3.3 V | 模块供电 |
| `GND` | GND | **共地** |

- **不复用循迹那条 I2C0（PA0/PA1）**：PA0/PA1 保持给灰度，两边互不影响、可同时接。
- 7 位地址 **0x5A**（固件上电会 `0x5A`/`0x00` 都试一遍）；速度约 **80 kHz**；读 3 字节并做 **CRC-8(PEC)** 校验 + 重试。
- **上拉**：SCL 由 MCU 推挽驱动，不需要上拉；**SDA 建议外接 4.7 k 到 3.3 V**（固件内部上拉约 40 k 偏弱，读数不稳就把 `MLX_SOFT_HALF_US` 调大，或加这个 4.7 k）。
- 走线短（软 I2C 抗干扰弱）；远离电机 PWM 线。

### 3.5 授时 ESP32-S2 从机（硬件 I2C1，0x42）

| 信号 | 引脚 |
|---|---|
| SDA | PA30 |
| SCL | PA29 |

- 400 kHz，7 位地址 **0x42**，**纯读 15 字节**（`buf[0]=0x01` 有效 + `YYYYMMDDhhmmss`）。
- 上拉 **2.2 k~3.3 k** 到 3.3 V；**模块单独 3.3 V 供电并共地**（峰值电流大）；天线上方别铺铜；走线 ≤10 cm。

### 3.6 串口占用

| 实例 | 引脚 | 波特率 | 用途 | 备注 |
|---|---|---|---|---|
| UART0 | TX=PA10 | 115200 | 语音 ASRPRO2.0（只发不接） | RX=PA11 空着 |
| UART1 | TX=PA8 / RX=PA9 | 115200 | IMU（只收） | 模块 25 Hz 自动上报 |
| UART2 | TX=PB15 / RX=PB16 | 115200 | 空闲 | 预留 |
| **UART3** | **TX=PB2 / RX=PB3** | **9600** | **LORA（= `printf` 出口）** | |

- **接线一律交叉 + 共地**：MCU TX → 模块 RX，MCU RX ← 模块 TX。
- **语音 ASRPRO2.0**：`PA10 → ASRPRO 串口1 RX`（模块上那颗 **PA3**，**别用它的串口0=PB5/PB6，那是下载口**）；115200-8-N-1；模块 3.6~5 V、>500 mA、外接喇叭。命令是**裸 ASCII 串、无结束符**：`M1..M4`（开始测温）/`D1..D4`（测完）/`E`（任务完成）。
- **LORA**：433 MHz 模组，`PB2→RXD`、`PB3→TXD`、3V3、GND。电脑侧另一个 LORA + USB-TTL（交叉 + 共地），串口助手 9600，**两端信道/空中速率/地址必须一致**。
- ⚠️ **别照 `middle/LORA.h` 里的 `GPIO_LORA_TX = GPIO_PIN_8 (GPIOA)` 接**——STM32 残留，照它接会把 LORA 接到 **PA8（IMU 的 TX）**。以本表为准：**PB2/PB3**。

### 3.7 LCD（ST7735S 128×160，硬件 SPI1）

| 信号 | 引脚 |
|---|---|
| SCLK | PB9 |
| SDA/MOSI | PB8 |
| RES | PB10 |
| DC | PB11 |
| CS | PB14 |
| BLK | PB26 |

- SPI1，`MOTO3`、PICO-only（**无 MISO**），target 32 MHz → **走线尽量短**，SCLK 串 22~33 Ω 更稳。
- 3.3 V、共地；BLK 背光按模块要求（有的需限流电阻）。

### 3.8 电池电压检测（ADC0/PA27）

- 12 V 经 **1/4 分压** → PA27；ADC 12 bit，VDDA 参考，采样时间 10 ms，重复采样/软件触发。
- 固件电量公式：`% = (Vbat − 10.5) / (12.6 − 10.5) × 100`。**分压比与端点要按实际电池上台标定**。
- 分压节点并 100 nF 到地抗混叠；上臂阻值 ≤ 100 k，减小 ADC 源阻抗影响。

### 3.9 按键 / 其它

| 功能 | 引脚 | 说明 |
|---|---|---|
| S2（唯一在用） | PB21 | 内部上拉，**按下接 GND**；单击=启停，长按=调速 |
| S1 | PA18 | **弃用**（复位时为高进 BSL）；保留为 BSL 按键或拉低 |
| 空闲 | PA16 / PB15 / PB16 / PA11 | HC05 STATE、UART2、UART0 RX（预留） |

---

## 4. 上拉 / 去耦 / 保护清单

| 位置 | 元件 | 说明 |
|---|---|---|
| I2C0 SDA/SCL | 2.2~4.7 k ×2 到 3.3 V | **必需**（灰度总线） |
| MLX90614 SDA (PB18) | 4.7 k 到 3.3 V | **建议**（固件内部上拉 ~40 k 偏弱） |
| MLX90614 SCL (PB17) | 不需上拉 | MCU 推挽驱动 |
| I2C1 SDA/SCL | 2.2~3.3 k ×2 到 3.3 V | **必需** |
| 编码器 A/B 相 ×8 | 10 k 到 3.3 V | 仅开漏编码器需要 |
| PB0 (STBY) | 10 k 下拉到 GND | 复位期间电机待机（推荐） |
| TB6612 VM | 100 µF + 100 nF ×2 片 | 就近放在 VM 脚 |
| MCU VDD | 每脚 100 nF；公共 10 µF | 见 §6 |
| LCD SCLK | 22~33 Ω 串阻 | 32 MHz 抗振铃（可选） |

---

## 5. PCB 注意事项（重点）

1. **12 V 绝不进 MCU / 3.3 V**；上电前万用表量 VM-GND、VCC-GND 不短路。
2. **电机地 / 逻辑地单点汇接**：电机大电流不要流过 MCU 和传感器的地（星型地或 0 Ω 桥接），否则复位/读数抖动。
3. **两片 TB6612 的 VCC 必须 3.3 V**（门限问题见 §1）。
4. **SWD 要留**：PA19(SWDIO)/PA20(SWCLK)/GND/3V3 引 4 脚排针；否则后续烧录/调试要飞线。
5. **NRST 复位**：按数据手册放 10 k 上拉 +（可选）1 nF 到地 + 复位按键；复位脚别被长期拉低。
6. **BSL 引脚 PA18**：复位期间为高会进 Bootloader。不用就**拉低**，要 BSL 才对地接按键。
7. **VCORE / VDDA 支持电路**（自绘裸片时必须）：VCORE 是内核 LDO 输出，要按 **数据手册**放去耦电容；VDDA 单独 100 nF+1 µF，最好经磁珠/0 Ω 从 VDD 引入。**容值以 TI《MSPM0G3507 数据手册 / 硬件设计指南》为准。**
8. **无外部晶振**：本工程用内部 SYSOSC + PLL，自绘板可省晶振。
9. **软 I2C 的 PB17/PB18 走线要短**、远离电机 PWM 与 VM 大电流回路（软 I2C 抗扰弱、内部上拉弱）。
10. **ADC 分压节点**（PA27）远离 PWM/电机走线；见 §3.8。
11. **I2C 总线一条一组上拉**：不要每个模块各带一组（并联后阻值过低）。
12. **LORA / 授时模块天线上方禁铺铜**，尽量靠板边。
13. **外形限制**：整车（含 PCB）长×宽×高 ≤ **25 cm × 25 cm × 25 cm**，画板留余量。
14. **接插防呆**：4 电机 + 4 组编码器共 8 组线，容易插错；用防呆端子并标丝印。

---

## 6. 板形态：天猛星载板（已定）

**方案 = 载板**：一块 **天猛星 MSPM0G3507** 开发板插在载板上；**MCU 最小系统（VDD/VDDA/VCORE 去耦、NRST、SWD、BSL、时钟）全由天猛星板提供**，载板不用再搭。

载板要做的事：

- **电源**：12 V 电池 → 一路给两片 TB6612 的 `VM`；一路做 **12 V→3.3 V**（给 TB6612 逻辑 + 各模块）；天猛星板自身供电（按其说明，常见 3.3 V 或 5 V/USB）。
- **两片 TB6612** + 电机/编码器端子。
- **各模块座**：灰度 I2C0、MLX90614 软 I2C、IMU UART1、LCD SPI、LORA UART3、授时 I2C1、语音 UART0。
- **对接排针**：按天猛星板的**排针定义**画 —— 先拿到它的原理图/丝印，确认哪根脚是 PA0/PA1/PA17… 再落板。

> MCU 在现成板上，所以 §5 第 5~8 条（NRST / VCORE / VDDA / 晶振）**由天猛星板负责**；§5 其余条（地、电源、上拉、天线、走线）依旧适用。
> ⚠️ 天猛星板的 3.3 V/5 V 输出**带载能力有限**：授时模块、语音模块这类大电流模块建议**独立供电**，别从天猛星板上取。

---

## 7. 与固件对应（方向/极性错了改宏，不拆线）

| 现象 | 改哪里 |
|---|---|
| 某轮转向与指令相反 | `app/Motor.c` `MOTOR_x_REVERSE` |
| 编码器实测速度为负/抖 | `app/Encoder.c` `ENC_x_REVERSE` |
| 灰度探头左右装反 | `middle/XunJi.c` `TRACK_SENSOR_FLIP` |
| 纠正方向反 | `middle/XunJi.c` `TRACK_LR_SWAP` |
| 白底数值比黑线还大（极性反） | `middle/XunJi.c` `TRACK_DARK_IS_HIGH` |
| IMU 转向方向反 | `middle/ImuTurn.h` `IMU_TURN_YAW_SIGN` |
| MLX90614 地址不对 | `app/MLX90614.c` `MLX90614_I2C_ADDR`（或调软 I2C 半位延时 `MLX_SOFT_HALF_US`） |
| 电池分压/量程不符 | `empty.c` `BAT_DIV_RATIO` / `BAT_EMPTY_V` / `BAT_FULL_V` |
| LORA 口/波特率 | SysConfig 的 `LORA`（UART3）；`printf` 出口在 `middle/UART.c` `fputc()` |

### 固件清理（2026-10-14 已做）

- `app/TW.c` 的 `TW_Init()` 原来会清 PB17/PB18（老“语音控制线”）——**已删**；这两个脚现在是 MLX90614 的 SCL/SDA，`TW_Init()` 改成空实现并留了警告注释。
- `empty.c` 里的 `MLX90614_I2C_100K`（把 I2C0 降到 100 kHz 的硬件 I2C0 老方案残留）——**已删**。
- 改完已 Rebuild：**0 error / 0 warning**。

> 原则：**注释不可信、以实测为准**。改 `.c/.h` 后要 **Rebuild**；改 `.uvprojx/.syscfg` 前先关 uVision。
