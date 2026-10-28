#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate the DZZH carrier-board hardware package (EDA-agnostic).

Outputs (pcb/):
  connection_matrix.csv   module pin <-> TMX signal, one row per connection
  netlist.txt             net -> endpoints (TMX.PA0, J11.SDA, ...)
  BOM.csv                 carrier-board BOM + plug-in modules
  validation_report.txt   cross-check against ti_msp_dl_config.h

Everything is derived from ti_msp_dl_config.h + the module pin maps below.
"""

from pathlib import Path
import csv
import re

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "pcb"
OUT.mkdir(exist_ok=True)

# ----------------------------------------------------------------------------
# TMX (天猛星 MSPM0G3507) logical pin map.  Physical header order is assigned
# in the PCB after a continuity check; the netlist only depends on these names.
# ----------------------------------------------------------------------------
TMX_PINS = {
    "PA0":  ("I2C0_SDA",   "循迹灰度 SDA"),
    "PA1":  ("I2C0_SCL",   "循迹灰度 SCL"),
    "PA7":  ("M_D_PWM",    "TB6612#2 PWMB (D 左后)"),
    "PA8":  ("IMU_TX",     "UART1 TX -> IMU RX"),
    "PA9":  ("IMU_RX",     "UART1 RX <- IMU TX"),
    "PA10": ("VOICE_TX",   "UART0 TX -> ASRPRO RX"),
    "PA11": ("NC",         "UART0 RX 空置"),
    "PA12": ("M_A_PWM",    "TB6612#1 PWMA (A 左前)"),
    "PA13": ("M_B_PWM",    "TB6612#1 PWMB (B 右前)"),
    "PA14": ("ENC_D_A",    "编码器 D 左后 A 相"),
    "PA15": ("ENC_D_B",    "编码器 D 左后 B 相"),
    "PA16": ("KEY_K1",     "按键 K1"),
    "PA17": ("M_C_PWM",    "TB6612#2 PWMA (C 右后)"),
    "PA18": ("NC",         "BSL，保持低/悬空"),
    "PA19": ("SWDIO",      "SWD 调试"),
    "PA20": ("SWCLK",      "SWD 调试"),
    "PA21": ("ENC_A_A",    "编码器 A 左前 A 相（天猛星特殊脚，需核对）"),
    "PA24": ("ENC_C_A",    "编码器 C 右后 A 相"),
    "PA25": ("ENC_C_B",    "编码器 C 右后 B 相"),
    "PA26": ("ENC_A_B",    "编码器 A 左前 B 相"),
    "PA27": ("VBAT_SENSE", "电池 1/4 分压 ADC"),
    "PA28": ("ENC_B_B",    "编码器 B 右前 B 相"),
    "PA29": ("I2C1_SCL",   "授时 SCL"),
    "PA30": ("I2C1_SDA",   "授时 SDA"),
    "PA31": ("ENC_B_A",    "编码器 B 右前 A 相"),
    "PB0":  ("STBY",       "两片 TB6612 STBY"),
    "PB1":  ("KEY_K2",     "按键 K2"),
    "PB2":  ("LORA_TX",    "UART3 TX -> LORA RXD"),
    "PB3":  ("LORA_RX",    "UART3 RX <- LORA TXD"),
    "PB4":  ("M_B_IN1",    "TB6612#1 BIN1"),
    "PB5":  ("M_B_IN2",    "TB6612#1 BIN2"),
    "PB6":  ("M_C_IN1",    "TB6612#2 AIN1"),
    "PB7":  ("M_C_IN2",    "TB6612#2 AIN2"),
    "PB8":  ("LCD_MOSI",   "SPI1 PICO -> LCD SDA"),
    "PB9":  ("LCD_SCLK",   "SPI1 SCLK"),
    "PB10": ("LCD_RES",    "LCD RESET"),
    "PB11": ("LCD_DC",     "LCD DC"),
    "PB12": ("M_D_IN1",    "TB6612#2 BIN1"),
    "PB13": ("M_D_IN2",    "TB6612#2 BIN2"),
    "PB14": ("LCD_CS",     "LCD CS"),
    "PB15": ("UART2_TX",   "UART2 预留"),
    "PB16": ("UART2_RX",   "UART2 预留"),
    "PB17": ("MLX_SCL",    "MLX90614 软件 I2C SCL"),
    "PB18": ("MLX_SDA",    "MLX90614 软件 I2C SDA"),
    "PB19": ("M_A_IN1",    "TB6612#1 AIN1"),
    "PB20": ("M_A_IN2",    "TB6612#1 AIN2"),
    "PB21": ("KEY_S2",     "S2 按键 -> GND（内部上拉）"),
    "PB23": ("KEY_K3",     "按键 K3"),
    "PB26": ("LCD_BLK",    "LCD 背光"),
    "3V3":  ("NC",         "天猛星板载 3.3V 输出：不接载板 +3V3（防倒灌）"),
    "5V":   ("TMX_5V",     "天猛星 5V 输入（经 D2 防倒灌）"),
    "GND":  ("GND",        "地"),
}

# ----------------------------------------------------------------------------
# Components.  Each entry: (ref, name, footprint_hint, [(pin, net, note), ...])
# "NC" pins are listed for documentation but skipped in the netlist.
# ----------------------------------------------------------------------------
COMPONENTS = [
    # ---- power input ----
    ("J1", "12V 电池输入", "KF301-2P / XT30", [
        ("1", "VBAT", "电池 +"), ("2", "GND", "电池 -")]),
    ("F1", "自恢复保险丝 5A", "1206 / 插件", [
        ("1", "VBAT", "输入"), ("2", "VBAT_F", "输出")]),
    ("D1", "防反接 P-MOSFET", "SOT-23 (AO3401)", [
        ("S", "VBAT_F", "源"), ("D", "VM_12V", "漏"), ("G", "GND", "栅")]),
    ("D2", "SS34 防倒灌", "SMA", [
        ("A", "+5V", "载板 5V"), ("K", "TMX_5V", "给天猛星供电")]),
    ("TVS1", "TVS SMBJ15A", "SMB", [
        ("1", "VM_12V", "12V 钳位"), ("2", "GND", "")]),
    ("U1", "12V->3.3V 降压模块 3A", "模块 4P 2.54", [
        ("IN+", "VM_12V", ""), ("IN-", "GND", ""),
        ("OUT+", "+3V3", "逻辑 3.3V"), ("OUT-", "GND", "")]),
    ("U2", "12V->5V 降压模块 2A", "模块 4P 2.54", [
        ("IN+", "VM_12V", ""), ("IN-", "GND", ""),
        ("OUT+", "+5V", "语音 5V"), ("OUT-", "GND", "")]),
    ("U3", "5V->3.3V LDO 模块 1A", "模块 3P 2.54", [
        ("IN", "+5V", ""), ("GND", "GND", ""), ("OUT", "+3V3_T", "授时专用")]),
    # ---- decoupling / bulk ----
    ("C1", "100uF 电解", "6.3x7.7 / 插件", [
        ("1", "VM_12V", "TB6612 VM 储能"), ("2", "GND", "")]),
    ("C2", "100nF 瓷片", "0603", [("1", "VM_12V", ""), ("2", "GND", "")]),
    ("C3", "10uF 陶瓷", "0805", [("1", "+3V3", ""), ("2", "GND", "")]),
    ("C4", "100nF 瓷片", "0603", [("1", "+3V3", ""), ("2", "GND", "")]),
    ("C5", "10uF 陶瓷", "0805", [("1", "+5V", ""), ("2", "GND", "")]),
    ("C6", "100nF 瓷片", "0603", [("1", "VBAT_SENSE", "ADC 抗混叠"), ("2", "GND", "")]),
    # ---- battery sense 1/4 ----
    ("R1", "30k 1%", "0603", [("1", "VM_12V", "上臂"), ("2", "VBAT_SENSE", "")]),
    ("R2", "10k 1%", "0603", [("1", "VBAT_SENSE", "下臂"), ("2", "GND", "")]),
    # ---- I2C pull-ups ----
    ("R3", "4.7k", "0603", [("1", "+3V3", ""), ("2", "I2C0_SDA", "灰度上拉")]),
    ("R4", "4.7k", "0603", [("1", "+3V3", ""), ("2", "I2C0_SCL", "灰度上拉")]),
    ("R5", "2.2k", "0603", [("1", "+3V3", ""), ("2", "I2C1_SDA", "授时上拉")]),
    ("R6", "2.2k", "0603", [("1", "+3V3", ""), ("2", "I2C1_SCL", "授时上拉")]),
    ("R7", "4.7k", "0603", [("1", "+3V3", ""), ("2", "MLX_SDA", "软 I2C 上拉")]),
    # ---- power LED ----
    ("R8", "1k", "0603", [("1", "+3V3", ""), ("2", "LED_A", "")]),
    ("LED1", "电源指示 LED", "0603", [("A", "LED_A", ""), ("K", "GND", "")]),
    # ---- motor drivers (modules) ----
    ("J3", "TB6612 #1 模块 (A/B)", "模块 2x8 2.54 + 接线端子", [
        ("VM", "VM_12V", ""), ("VCC", "+3V3", "逻辑电源"),
        ("GND", "GND", ""), ("STBY", "STBY", ""),
        ("PWMA", "M_A_PWM", ""), ("AIN1", "M_A_IN1", ""), ("AIN2", "M_A_IN2", ""),
        ("PWMB", "M_B_PWM", ""), ("BIN1", "M_B_IN1", ""), ("BIN2", "M_B_IN2", ""),
        ("AO1", "MA_OUT1", "A 电机"), ("AO2", "MA_OUT2", "A 电机"),
        ("BO1", "MB_OUT1", "B 电机"), ("BO2", "MB_OUT2", "B 电机")]),
    ("J4", "TB6612 #2 模块 (C/D)", "模块 2x8 2.54 + 接线端子", [
        ("VM", "VM_12V", ""), ("VCC", "+3V3", "逻辑电源"),
        ("GND", "GND", ""), ("STBY", "STBY", ""),
        ("PWMA", "M_C_PWM", ""), ("AIN1", "M_C_IN1", ""), ("AIN2", "M_C_IN2", ""),
        ("PWMB", "M_D_PWM", ""), ("BIN1", "M_D_IN1", ""), ("BIN2", "M_D_IN2", ""),
        ("AO1", "MC_OUT1", "C 电机"), ("AO2", "MC_OUT2", "C 电机"),
        ("BO1", "MD_OUT1", "D 电机"), ("BO2", "MD_OUT2", "D 电机")]),
    # ---- encoders ----
    ("J5", "编码器 A 左前", "XH2.54-5P", [
        ("VCC", "+3V3", ""), ("GND", "GND", ""),
        ("A", "ENC_A_A", ""), ("B", "ENC_A_B", ""), ("Z", "NC", "")]),
    ("J6", "编码器 B 右前", "XH2.54-5P", [
        ("VCC", "+3V3", ""), ("GND", "GND", ""),
        ("A", "ENC_B_A", ""), ("B", "ENC_B_B", ""), ("Z", "NC", "")]),
    ("J7", "编码器 C 右后", "XH2.54-5P", [
        ("VCC", "+3V3", ""), ("GND", "GND", ""),
        ("A", "ENC_C_A", ""), ("B", "ENC_C_B", ""), ("Z", "NC", "")]),
    ("J8", "编码器 D 左后", "XH2.54-5P", [
        ("VCC", "+3V3", ""), ("GND", "GND", ""),
        ("A", "ENC_D_A", ""), ("B", "ENC_D_B", ""), ("Z", "NC", "")]),
    # ---- sensors / modules ----
    ("J9",  "循迹灰度 AuroraT8", "XH2.54-4P", [
        ("VCC", "+3V3", ""), ("GND", "GND", ""),
        ("SDA", "I2C0_SDA", ""), ("SCL", "I2C0_SCL", "")]),
    ("J10", "MLX90614 测温", "XH2.54-4P", [
        ("VIN", "+3V3", ""), ("GND", "GND", ""),
        ("SCL", "MLX_SCL", ""), ("SDA", "MLX_SDA", "")]),
    ("J11", "IMU (YbIMU/JY61P)", "XH2.54-4P", [
        ("VCC", "+3V3", ""), ("GND", "GND", ""),
        ("TX", "IMU_RX", "模块 TX -> MCU RX (PA9)"),
        ("RX", "IMU_TX", "模块 RX <- MCU TX (PA8)")]),
    ("J12", "语音 ASRPRO2.0", "XH2.54-4P", [
        ("5V", "+5V", "3.6~5V"), ("GND", "GND", ""),
        ("RX", "VOICE_TX", "模块 RX <- MCU TX"), ("TX", "NC", "未用")]),
    ("J13", "LORA 433MHz", "XH2.54-4P", [
        ("VCC", "+3V3", ""), ("GND", "GND", ""),
        ("TXD", "LORA_RX", "模块 TX -> MCU RX"), ("RXD", "LORA_TX", "模块 RX <- MCU TX")]),
    ("J14", "授时 ESP32-S2", "XH2.54-4P", [
        ("3V3", "+3V3_T", "独立供电"), ("GND", "GND", ""),
        ("SDA", "I2C1_SDA", ""), ("SCL", "I2C1_SCL", "")]),
    ("J15", "LCD ST7735S 128x160", "XH2.54-8P", [
        ("VCC", "+3V3", ""), ("GND", "GND", ""),
        ("SCLK", "LCD_SCLK", ""), ("MOSI", "LCD_MOSI", ""),
        ("RES", "LCD_RES", ""), ("DC", "LCD_DC", ""),
        ("CS", "LCD_CS", ""), ("BLK", "LCD_BLK", "")]),
    # ---- user IO ----
    ("SW1", "S2 按键", "6x6 轻触", [
        ("1", "KEY_S2", ""), ("2", "GND", "")]),
    ("K1", "按键 K1", "6x6 轻触", [
        ("1", "KEY_K1", ""), ("2", "GND", "")]),
    ("K2", "按键 K2", "6x6 轻触", [
        ("1", "KEY_K2", ""), ("2", "GND", "")]),
    ("K3", "按键 K3", "6x6 轻触", [
        ("1", "KEY_K3", ""), ("2", "GND", "")]),
    ("J16", "SWD 调试口", "XH2.54-4P / 2.54 排针", [
        ("3V3", "+3V3", ""), ("GND", "GND", ""),
        ("SWDIO", "SWDIO", ""), ("SWCLK", "SWCLK", "")]),
    ("J17", "UART2 预留", "XH2.54-4P", [
        ("3V3", "+3V3", ""), ("GND", "GND", ""),
        ("TX", "UART2_TX", ""), ("RX", "UART2_RX", "")]),
]


def net_ends():
    ends = {}
    for ref, _name, _fp, pins in COMPONENTS:
        for pn, net, _note in pins:
            if net in ("", "NC"):
                continue
            ends.setdefault(net, []).append(f"{ref}.{pn}")
    for pin, (net, _note) in TMX_PINS.items():
        if net in ("", "NC"):
            continue
        ends.setdefault(net, []).append(f"TMX.{pin}")
    return ends


def direction(net, pin, module_name):
    if net in ("I2C0_SDA", "I2C0_SCL", "I2C1_SDA", "I2C1_SCL", "MLX_SDA", "MLX_SCL"):
        return "<->"
    if net.startswith(("M_", "STBY")) or net.startswith("ENC_") or net in ("KEY_S2", "VBAT_SENSE"):
        if net.startswith("M_") and net.endswith(("_PWM", "_IN1", "_IN2")):
            return "->"
        if net.startswith("ENC_"):
            return "->"
        return "->"
    if net in ("IMU_TX", "VOICE_TX", "LORA_TX", "LCD_SCLK", "LCD_MOSI", "LCD_RES", "LCD_DC", "LCD_CS", "LCD_BLK", "UART2_TX"):
        return "->"
    if net in ("IMU_RX", "LORA_RX", "UART2_RX"):
        return "<-"
    if net in ("SWDIO", "SWCLK"):
        return "<->"
    if net.startswith("+") or net in ("GND", "VBAT", "VM_12V", "VBAT_F", "VBAT_SENSE"):
        return "-"
    return "-"


def main():
    ends = net_ends()
    tmx_of_net = {net: pin for pin, (net, _n) in TMX_PINS.items()}

    # ---- connection matrix ----
    rows = []
    for ref, name, _fp, pins in COMPONENTS:
        for pn, net, note in pins:
            if net in ("", "NC"):
                continue
            rows.append({
                "Block": name,
                "Net": net,
                "TMX_Signal": tmx_of_net.get(net, "POWER" if net.startswith("+") or net in ("GND", "VBAT", "VM_12V", "VBAT_F", "VBAT_SENSE") else "LOCAL"),
                "Module": ref,
                "Module_Pin": pn,
                "Dir": direction(net, pn, name),
                "Note": note,
            })
    with open(OUT / "connection_matrix.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["Block", "Net", "TMX_Signal", "Module", "Module_Pin", "Dir", "Note"])
        w.writeheader()
        w.writerows(rows)

    # ---- netlist ----
    lines = ["# DZZH carrier board logical netlist",
             "# TMX.<pin> = 天猛星逻辑引脚; Jn.<pin> = 载板连接器",
             ""]
    for net in sorted(ends):
        lines.append(f"NET {net}")
        for e in ends[net]:
            lines.append(f"  {e}")
        lines.append("")
    (OUT / "netlist.txt").write_text("\n".join(lines), encoding="utf-8")

    # ---- BOM ----
    bom = [
        ("1", "天猛星 MSPM0G3507 开发板", "TMX", "插件（双排母，按实物）", "主控"),
        ("2", "TB6612FNG 电机驱动模块", "J3,J4", "2x8 2.54 + 端子", "A/B、C/D"),
        ("1", "芯流 AuroraT8 八路灰度", "J9", "4P", "I2C0 0x40"),
        ("1", "MLX90614 测温模块", "J10", "4P", "软 I2C PB17/PB18"),
        ("1", "IMU 模块 (YbIMU/JY61P)", "J11", "4P", "UART1"),
        ("1", "LORA 433MHz 模块", "J13", "4P", "UART3 9600"),
        ("1", "ESP32-S2 授时模块", "J14", "4P", "I2C1 0x42"),
        ("1", "ASRPRO2.0 语音模块 + 喇叭", "J12", "4P", "UART0"),
        ("1", "ST7735S 128x160 LCD", "J15", "8P", "SPI1"),
        ("4", "编码器电机 / 编码器模块", "J5-J8", "5P", "A/B 相"),
        ("1", "12V->3.3V 降压模块 3A", "U1", "4P", ""),
        ("1", "12V->5V 降压模块 2A", "U2", "4P", ""),
        ("1", "5V->3.3V LDO 模块 1A", "U3", "3P", "授时独立供电"),
        ("1", "自恢复保险丝 5A", "F1", "1206/插件", ""),
        ("1", "P-MOSFET AO3401", "D1", "SOT-23", "防反接"),
        ("1", "SS34 肖特基", "D2", "SMA", "天猛星 5V 防倒灌"),
        ("1", "TVS SMBJ15A", "TVS1", "SMB", ""),
        ("1", "100uF 电解", "C1", "6.3x7.7", ""),
        ("1", "10uF 陶瓷", "C3", "0805", ""),
        ("1", "10uF 陶瓷", "C5", "0805", ""),
        ("3", "100nF 瓷片", "C2,C4,C6", "0603", ""),
        ("2", "30k / 10k 1%", "R1,R2", "0603", "1/4 分压"),
        ("2", "4.7k", "R3,R4", "0603", "I2C0 上拉"),
        ("2", "2.2k", "R5,R6", "0603", "I2C1 上拉"),
        ("1", "4.7k", "R7", "0603", "MLX SDA 上拉"),
        ("1", "1k", "R8", "0603", "LED 限流"),
        ("1", "LED 3.3V 指示", "LED1", "0603", ""),
        ("1", "S2 轻触按键", "SW1", "6x6", ""),
        ("3", "额外轻触按键 K1/K2/K3", "K1,K2,K3", "6x6", "PA16/PB1/PB23，内部上拉"),
        ("1", "4P SWD 排针", "J16", "2.54", ""),
        ("1", "4P 预留 UART2", "J17", "2.54", ""),
    ]
    with open(OUT / "BOM.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["Qty", "Item", "Ref", "Footprint", "Note"])
        w.writerows(bom)

    # ---- validation against ti_msp_dl_config.h ----
    hdr = (ROOT / "ti_msp_dl_config.h").read_text(encoding="gbk", errors="replace")
    defs = dict(re.findall(r"^\s*#define\s+(\w+)\s+([^\r\n/]+)", hdr, re.M))
    checks = [
        ("GPIO_I2C_0_SDA_PORT", "GPIOA"), ("GPIO_I2C_0_SDA_PIN", "DL_GPIO_PIN_0"),
        ("GPIO_I2C_0_SCL_PORT", "GPIOA"), ("GPIO_I2C_0_SCL_PIN", "DL_GPIO_PIN_1"),
        ("GPIO_I2C_TIME_SDA_PORT", "GPIOA"), ("GPIO_I2C_TIME_SDA_PIN", "DL_GPIO_PIN_30"),
        ("GPIO_I2C_TIME_SCL_PORT", "GPIOA"), ("GPIO_I2C_TIME_SCL_PIN", "DL_GPIO_PIN_29"),
        ("GPIO_UART_0_TX_PORT", "GPIOA"), ("GPIO_UART_0_TX_PIN", "DL_GPIO_PIN_10"),
        ("GPIO_IMU_TX_PORT", "GPIOA"), ("GPIO_IMU_TX_PIN", "DL_GPIO_PIN_8"),
        ("GPIO_IMU_RX_PORT", "GPIOA"), ("GPIO_IMU_RX_PIN", "DL_GPIO_PIN_9"),
        ("GPIO_LORA_TX_PORT", "GPIOB"), ("GPIO_LORA_TX_PIN", "DL_GPIO_PIN_2"),
        ("GPIO_LORA_RX_PORT", "GPIOB"), ("GPIO_LORA_RX_PIN", "DL_GPIO_PIN_3"),
        ("GPIO_Motor_PWM_Front_C0_PORT", "GPIOA"), ("GPIO_Motor_PWM_Front_C0_PIN", "DL_GPIO_PIN_12"),
        ("GPIO_Motor_PWM_Front_C1_PORT", "GPIOA"), ("GPIO_Motor_PWM_Front_C1_PIN", "DL_GPIO_PIN_13"),
        ("GPIO_Motor_PWM_Back_C0_PORT", "GPIOA"), ("GPIO_Motor_PWM_Back_C0_PIN", "DL_GPIO_PIN_17"),
        ("GPIO_Motor_PWM_Back_C1_PORT", "GPIOA"), ("GPIO_Motor_PWM_Back_C1_PIN", "DL_GPIO_PIN_7"),
        ("GPIO_ADC_VOLTAGE_C0_PORT", "GPIOA"), ("GPIO_ADC_VOLTAGE_C0_PIN", "DL_GPIO_PIN_27"),
        ("GPIO_LCD_DATA_PICO_PORT", "GPIOB"), ("GPIO_LCD_DATA_PICO_PIN", "DL_GPIO_PIN_8"),
        ("GPIO_LCD_DATA_SCLK_PORT", "GPIOB"), ("GPIO_LCD_DATA_SCLK_PIN", "DL_GPIO_PIN_9"),
        ("LCD_PORT", "GPIOB"), ("LCD_LCD_RES_PIN", "DL_GPIO_PIN_10"),
        ("LCD_LCD_DC_PIN", "DL_GPIO_PIN_11"), ("LCD_LCD_CS_PIN", "DL_GPIO_PIN_14"),
        ("LCD_BLK_PIN", "DL_GPIO_PIN_26"),
        ("Motor_PORT", "GPIOB"), ("Motor_AIN1_PIN", "DL_GPIO_PIN_19"),
        ("Motor_AIN2_PIN", "DL_GPIO_PIN_20"), ("Motor_BIN1_PIN", "DL_GPIO_PIN_4"),
        ("Motor_BIN2_PIN", "DL_GPIO_PIN_5"), ("Motor_CIN1_PIN", "DL_GPIO_PIN_6"),
        ("Motor_CIN2_PIN", "DL_GPIO_PIN_7"), ("Motor_DIN1_PIN", "DL_GPIO_PIN_12"),
        ("Motor_DIN2_PIN", "DL_GPIO_PIN_13"), ("Motor_STBY_PIN", "DL_GPIO_PIN_0"),
        ("ENCODER_PORT", "GPIOA"), ("ENCODER_E1A_PIN", "DL_GPIO_PIN_21"),
        ("ENCODER_E1B_PIN", "DL_GPIO_PIN_26"), ("ENCODER_E2A_PIN", "DL_GPIO_PIN_31"),
        ("ENCODER_E2B_PIN", "DL_GPIO_PIN_28"), ("ENCODER_E3A_PIN", "DL_GPIO_PIN_24"),
        ("ENCODER_E3B_PIN", "DL_GPIO_PIN_25"), ("ENCODER_E4A_PIN", "DL_GPIO_PIN_14"),
        ("ENCODER_E4B_PIN", "DL_GPIO_PIN_15"),
        ("KEY1_PORT", "GPIOB"), ("KEY1_UP_PIN", "DL_GPIO_PIN_21"),
        ("TW_PORT", "GPIOB"), ("TW_TW_0_PIN", "DL_GPIO_PIN_17"), ("TW_TW_1_PIN", "DL_GPIO_PIN_18"),
    ]
    report = []
    fails = 0
    for name, exp in checks:
        got = defs.get(name, "<missing>").strip()
        while got.startswith("(") and got.endswith(")"):
            got = got[1:-1].strip()
        ok = got == exp
        fails += 0 if ok else 1
        report.append(f"{'PASS' if ok else 'FAIL'}  {name:32s} expected {exp:20s} got {got}")

    # MLX90614 soft-I2C pins are hardcoded in the driver
    mlx = (ROOT / "app" / "MLX90614.c").read_text(encoding="gbk", errors="replace")
    mlx_ok = ("DL_GPIO_PIN_17" in mlx) and ("DL_GPIO_PIN_18" in mlx)
    fails += 0 if mlx_ok else 1
    report.append(f"{'PASS' if mlx_ok else 'FAIL'}  MLX90614.c soft-I2C uses PB17/PB18")

    report.insert(0, f"DZZH carrier validation: {len(checks)+1} checks, {fails} failure(s)\n")
    (OUT / "validation_report.txt").write_text("\n".join(report) + "\n", encoding="utf-8")

    print(f"wrote {OUT}")
    print(f"  connection_matrix.csv : {len(rows)} rows")
    print(f"  netlist.txt           : {len(ends)} nets")
    print(f"  BOM.csv               : {len(bom)} lines")
    print(f"  validation_report.txt : {fails} failure(s)")
    if fails:
        for line in report:
            if line.startswith("FAIL"):
                print("  " + line)


if __name__ == "__main__":
    main()
