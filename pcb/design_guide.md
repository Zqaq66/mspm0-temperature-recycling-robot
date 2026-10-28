# DZZH 载板设计方案（天猛星 MSPM0G3507 + 全模块）

> 配套文件（同目录）：
> - `connection_matrix.csv` —— 逐脚连接矩阵（135 行）
> - `netlist.txt` —— 逻辑网表（60 个网络）
> - `BOM.csv` —— 载板 BOM + 插件模块
> - `BOM_lcsc.csv` —— **标准 LCSC 料号**（已验证编号）
> - `validation_report.txt` —— 与 `ti_msp_dl_config.h` 的 62 项交叉校验（当前 0 失败）
> - `carrier_block_diagram.png` —— 连接框图
> - `DZZH_carrier.kicad_sch` —— **可编辑原理图**（KiCad 8，A0）
> - `DZZH_carrier.epro` —— **立创EDA专业版原生工程**（已用立创EDA自带转换器生成，导入最稳）
> - `DZZH_carrier_sch_preview.png` —— 原理图预览
>
> 生成脚本：`tools/gen_carrier_hw.py`、`tools/draw_carrier_block.ps1`

---

## 1. 板形态

- **载板（Carrier）**：一块 **天猛星 MSPM0G3507** 开发板插上去；MCU 最小系统、板载 3.3V、CH340、SWD、40MHz HFXT 都由天猛星板提供。
- **全部外设用现成模块**：TB6612×2、灰度、MLX90614、IMU、LORA、授时、语音、LCD、编码器。
- 载板只做：**电源 + 转接 + 上拉/去耦 + 接线端子**。
- **不要**在载板上再放 MCU、晶振、VCORE/NRST 电路。

---

## 2. 电源

| 轨 | 来源 | 负载 | 选型建议 |
|---|---|---|---|
| `VM_12V` | 12V 电池 → 5A 自恢复保险 → P-MOS 防反接 → TVS | 两片 TB6612 `VM` | 走线 ≥2mm；TB6612 旁 100µF+100nF |
| `+3V3` | U1：12V→3.3V 降压模块 | TB6612 VCC、灰度、MLX、IMU、LORA、LCD、编码器、上拉（**不含天猛星**） | **3A** 模块，留 2 倍余量 |
| `+5V` | U2：12V→5V 降压模块 | ASRPRO2.0 + **经 D2 给天猛星供电** | **2A** 模块 |
| `+3V3_T` | U3：5V→3.3V | 授时 ESP32-S2（峰值 300~500mA） | 建议**小降压模块**而非 AMS1117（0.5A×1.7V≈0.85W 会烫） |

估算：`+3V3` 稳态约 0.4A、峰值约 0.6A；`+5V` 峰值约 1A；`3V3_T` 峰值约 0.5A。

> ⚠️ **天猛星的 3V3 引脚是板载稳压器的输出，绝不能与载板 +3V3 并联**（会倒灌/打架）。
> 天猛星由载板 `+5V → D2(SS34) → TMX_5V` 供电；`TMX_5V` 同时接天猛星 5V 脚。若确认板上 5V 脚已有防倒灌二极管，D2 可短接。

**电池检测**：`VM_12V → R1 30k → PA27 → R2 10k → GND`，PA27 并 100nF。
固件公式：`% = (Vbat − 10.5) / (12.6 − 10.5) × 100`，端点按实际电池标定（`empty.c` 的 `BAT_*`）。

---

## 3. 连接器定义（载板侧）

> 下面是**逻辑定义**；实物模块的排针顺序可能不同（尤其 TB6612 模块有多个版本），落板前按实物核对。

| 位号 | 模块 | 引脚定义 |
|---|---|---|
| J1 | 12V 电池 | 1=VBAT, 2=GND |
| J3 | TB6612 #1 | VM, VCC, GND, STBY, PWMA, AIN1, AIN2, PWMB, BIN1, BIN2 + AO1/AO2/BO1/BO2 端子 |
| J4 | TB6612 #2 | 同上（C/D） |
| J5–J8 | 编码器 A/B/C/D | VCC, GND, A, B, Z(空) |
| J9 | 灰度 AuroraT8 | VCC, GND, SDA, SCL |
| J10 | MLX90614 | VIN, GND, SCL, SDA |
| J11 | IMU | VCC, GND, TX, RX |
| J12 | ASRPRO2.0 | 5V, GND, RX, TX(空) |
| J13 | LORA | VCC, GND, TXD, RXD |
| J14 | 授时 ESP32-S2 | 3V3, GND, SDA, SCL |
| J15 | LCD ST7735S | VCC, GND, SCLK, MOSI, RES, DC, CS, BLK |
| J16 | SWD | 3V3, GND, SWDIO, SWCLK |
| J17 | UART2 预留 | 3V3, GND, TX, RX |
| SW1 | S2 按键 | PB21 ↔ GND |

**串口一律交叉**：MCU TX → 模块 RX，MCU RX ← 模块 TX。

---

## 4. 上拉 / 去耦

| 网络 | 元件 | 位置 |
|---|---|---|
| I2C0（PA0/PA1，灰度） | 4.7k ×2 到 +3V3 | 靠近 J9 |
| I2C1（PA29/PA30，授时） | 2.2k ×2 到 +3V3 | 靠近 J14 |
| MLX SDA（PB18） | 4.7k 到 +3V3 | 靠近 J10（SCL 由 MCU 推挽，不需要） |
| 编码器 A/B ×8 | 10k 到 +3V3（仅开漏编码器） | 靠近 J5–J8 |
| PB0 STBY | 10k 下拉到 GND | 靠近 TMX 座（复位期间电机待机） |
| TB6612 VM | 100µF + 100nF ×2 | 每片模块旁 |
| 各模块 VCC | 100nF | 每个连接器旁 |

---

## 5. 布局建议

1. **天猛星座放板中/一侧**：USB-C 与 BSL/RST 按键朝板边，方便插拔和下载。
2. **电源入口（J1/F1/D1/TVS）靠板边**，12V 大电流回路短而粗。
3. **两片 TB6612 靠近对应的电机端子**，AO/BO 走线短；VM 走线 ≥2mm。
4. **天线类模块（LORA、授时）靠板边**，天线正下方**禁铺铜**。
5. **模拟/软 I2C 远离电机 PWM 和 VM**：灰度、MLX 的走线短，PB17/PB18 尤其要短。
6. **地**：电机回流与逻辑地**单点汇接**在电池负极附近；不要在 MCU 地上串电机电流。
7. **丝印**：每个连接器标 `Jx` + 功能 + 1 脚位置；4 组电机/编码器最容易插错。
8. **尺寸**：整车 ≤25×25×25cm；载板建议 ≤120×100mm，留出模块和插拔空间。

---

## 6. 天猛星板载资源 / 特殊脚（重要）

| 引脚 | 注意 |
|---|---|
| PA10 / PA11 | 板载 **CH340 的 UART0**。本项目用 PA10 发语音，等于同时送到 CH340 RX（只发不接，通常无碍）；PA11 空置。 |
| PA21 / PA23 / PA2 / PA18 | 厂家标注**特殊脚**，尽量别用。本项目编码器 A 用了 **PA21**，落板前确认板上没有冲突。 |
| PA18 | BSL 功能脚，复位为高会进 Bootloader；载板保持低/悬空。 |
| PA19 / PA20 | SWDIO / SWCLK，保留调试口。 |
| PA5 / PA6 | 板载 **40MHz HFXT**（80MHz 时钟源），**禁止复用**。 |
| PB22 | 板载用户 LED。 |
| 3V3 引脚 | **板载 3.3V 输出**，不要与载板 +3V3 并联（会倒灌）。 |
| 5V 引脚 | 天猛星供电输入：载板 +5V 经 D2(SS34) 送入，防止 USB/板载回路倒灌。 |

---

## 7. 上电与验证流程

1. **不插任何模块**，万用表量：VM-GND、+3V3-GND、+5V-GND 不短路。
2. 上 12V，测 `VM_12V≈12V`、`+3V3≈3.30V`、`+5V≈5.0V`、`+3V3_T≈3.30V`。
3. 插天猛星板，确认板载 LED/3V3 正常，能烧录。
4. **逐脚 continuity**：载板排针 ↔ 天猛星对应 GPIO（对照 `connection_matrix.csv`）。
5. 插灰度/MLX/IMU/LCD/LORA/授时，逐个上电看电流。
6. 烧固件，串口（LORA 9600）看 `==== TRACK mode ====` 与 `MLX ... scan/autodetect`。
7. 用 `MISSION_DEBUG_STEPS=4` 只跑到 1 号房间，确认 `WORK1` 的测温/电量/LORA 行。

---

## 8. 落板前仍需实物确认的 5 件事

1. **天猛星两排排针的物理顺序**（哪根是 PA0/PA1/PA17…）——用万用表从板上丝印/测试点核对；本方案按逻辑引脚名给出，不依赖顺序。
2. **TB6612 模块版本**——常见红板有 16P 排针版和端子版，引脚顺序不同；按你手上的模块改 `J3/J4` 定义。
3. **LCD / LORA / ASRPRO / 编码器模块的排针顺序**——按实物改连接器 1 脚。
4. **PA21 是否可用**——天猛星特殊脚；若不可用，把编码器 A 的 A 相换到其它空闲 GPIO，并同步改 `ti_msp_dl_config.h`/驱动。
5. **天猛星 5V 脚的方向**——用万用表确认它是电源输入还是 USB VBUS 输出；本方案按“输入 + D2 防倒灌”处理。

> 改完引脚后重新跑 `python tools/gen_carrier_hw.py`，`validation_report.txt` 会重新对一遍固件配置。

---

## 9. 原理图与导入立创EDA

**文件**

- `DZZH_carrier.kicad_sch`：KiCad 8 格式，A0 横向；**43 个器件、64 个网络、192 个网络标签**。
- `DZZH_carrier.epro`：立创EDA专业版工程（ZIP 容器），**推荐直接导入这个**。
- `_lceda_import.zip`：原理图 + 16 个 `.kicad_mod` 封装的源包（重新转换用）。
- `DZZH_carrier_sch_preview.png`：原理图预览（与 `.kicad_sch` 同一份数据渲染）。
- `DZZH_carrier_validation.txt`：回读 + 器件/网络/标签数量 + 几何重叠检查，当前 **PASS**。

**导入立创EDA专业版**

1. 打开立创EDA专业版 → **文件 → 导入 → 立创EDA专业版工程(.epro)**，选 `pcb/DZZH_carrier.epro`。
   （若菜单没有 .epro 入口，就用 **导入 → KiCad** 选 `pcb/DZZH_carrier.kicad_sch`。）
2. 导入后每个器件是一个“模块块”，引脚名就是信号名；**同名网络 NET 属性已经连好**。
3. 工程里已经带 **16 个真实封装**（`.epro` 的 `FOOTPRINT/*.efoo`），43 个器件全部有封装名。

> 这两份文件已用**立创EDA 自带的转换服务**（`convert-node-server.js`，`kicad → easyeda-pro`）无头验证：
> 返回 `{"success":true,"code":"success"}`，产物含 43 个位号、16 个封装、192 个网络标签、2 条说明文字。

**封装分配**

| 器件 | 封装 |
|---|---|
| 所有 4P 模块座（灰度/MLX/IMU/语音/LORA/授时/SWD/UART2） | `HDR-1X4` |
| 编码器 ×4 | `HDR-1X5` |
| LCD | `HDR-1X8` |
| TB6612 模块 ×2 | `HDR-2X7` |
| 电源模块 U1/U2 | `HDR-1X4`；U3 `HDR-1X3` |
| 电阻 / 0603 电容 / LED | `R0603` / `C0603` / `LED0603` |
| 0805 电容 | `C0805` |
| 肖特基/TVS | `SMA` |
| P-MOS | `SOT-23` |
| 保险丝 | `Fuse1206` |
| 电解电容 | `CAP-6.3x7.7` |
| 按键 S2/K1/K2/K3 | `SW-TH_6x6` |
| 电池/电源端子 | `KF301-2P` |

> ⚠️ `TMX` 的封装目前是占位的 `HDR-2X20`（40P）。天猛星实际排针更多/顺序不同，落板前请把它换成你实测的板级封装（两块 1×20 母座 + 电源座），并按实物核对网络。

### 标准 LCSC 料号（已验证）

| 用途 | LCSC | 型号 | 封装 |
|---|---|---|---|
| 10k 1% 0603 | C25804 | 0603WAF1002T5E | R0603 |
| 4.7k 1% 0603 | C23162 | 0603WAF4701T5E | R0603 |
| 1k 1% 0603 | C21190 | 0603WAF1001T5E | R0603 |
| 2.2k 1% 0603 | C4190 | 0603WAF2201T5E | R0603 |
| 100nF 0603 | C14663 | CC0603KRX7R9BB104 | C0603 |
| 10uF 0805 | C15850 | CL21A106KAYNNNE | C0805 |
| 100uF 35V | C72478 | RVT1V101M0607 | SMD D6.3×L7.7 |
| 红色 LED 0603 | C2286 | KT-0603R | LED0603 |
| SS34 | C8678 | SS34 | SMA |
| AO3401A | C15127 | AO3401A | SOT-23 |
| 2.54 1x3 | C124376 | 210S-1*3P | HDR-TH_3P |
| 2.54 1x4 | C124378 | Header-Male-2.54_1x4 | HDR-TH_4P |
| 2.54 1x5 | C124379 | 210S-1*5P | HDR-TH_5P |
| 2.54 1x8 | C124381 | Header-Male-2.54_1x8 | HDR-TH_8P |
| 2.54 2x8 | C124389 | 210S-2*8P | HDR-TH_16P |
| 2.54 2x20 | C124383 | 210S-2*20P | HDR-TH_40P |

> 30k / SMBJ15A / 6x6 按键 / KF301-2P / 5A 保险丝在 `BOM_lcsc.csv` 里标了“待定”，按封装选同规格即可。

**设计约定**

- 器件位号与 `connection_matrix.csv` / `BOM.csv` 完全一致（J1…J17、SW1、R1…R8、C1…C6、D1/D2、TVS1、U1…U3、LED1）。
- 所有连接用**同名 label** 表达；同名即同网，便于整体移动和改版。
- `NC` 引脚用 `no_connect` 标记，避免 ERC 误报。

**重新生成**

```powershell
python tools/gen_kicad_sch.py      # 生成 + 回读 + 几何/标签重叠校验
python tools/export_sch_json.py    # 导出预览数据
pwsh tools/render_sch_preview.ps1  # 渲染预览 PNG
```
