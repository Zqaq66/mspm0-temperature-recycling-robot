Add-Type -AssemblyName System.Drawing

$W = 2240; $H = 1140
$bmp = New-Object System.Drawing.Bitmap($W, $H)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$g.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::AntiAliasGridFit
$g.Clear([System.Drawing.Color]::White)

$fTitle = New-Object System.Drawing.Font("Microsoft YaHei", 20, [System.Drawing.FontStyle]::Bold)
$fH     = New-Object System.Drawing.Font("Microsoft YaHei", 11, [System.Drawing.FontStyle]::Bold)
$fB     = New-Object System.Drawing.Font("Microsoft YaHei", 9)
$fS     = New-Object System.Drawing.Font("Microsoft YaHei", 8)
$dark   = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(34, 34, 34))

function C([string]$s) { return [System.Drawing.ColorTranslator]::FromHtml($s) }

function Box($x0, $y0, $x1, $y1, $title, $lines, $color, $fill) {
    $pen = New-Object System.Drawing.Pen((C $color), 3)
    $br  = New-Object System.Drawing.SolidBrush((C $fill))
    $tbr = New-Object System.Drawing.SolidBrush((C $color))
    $g.FillRectangle($br, $x0, $y0, ($x1 - $x0), ($y1 - $y0))
    $g.DrawRectangle($pen, $x0, $y0, ($x1 - $x0), ($y1 - $y0))
    $g.DrawString($title, $fH, $tbr, ($x0 + 10), ($y0 + 6))
    $g.DrawString($lines, $fB, $dark, ($x0 + 12), ($y0 + 32))
    $pen.Dispose(); $br.Dispose(); $tbr.Dispose()
}

function Arrow($x1, $y1, $x2, $y2, $color, $label) {
    $pen = New-Object System.Drawing.Pen((C $color), 3)
    $g.DrawLine($pen, $x1, $y1, $x2, $y2)
    $ang = [Math]::Atan2(($y2 - $y1), ($x2 - $x1))
    $L = 14
    foreach ($s in @(-0.5, 0.5)) {
        $hx = $x2 - $L * [Math]::Cos($ang + $s)
        $hy = $y2 - $L * [Math]::Sin($ang + $s)
        $g.DrawLine($pen, $x2, $y2, $hx, $hy)
    }
    if ($label) {
        $mx = [int](($x1 + $x2) / 2); $my = [int](($y1 + $y2) / 2)
        $white = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::White)
        $g.FillRectangle($white, ($mx - 52), ($my - 25), 104, 20)
        $lbr = New-Object System.Drawing.SolidBrush((C $color))
        $g.DrawString($label, $fS, $lbr, ($mx - 50), ($my - 24))
        $white.Dispose(); $lbr.Dispose()
    }
    $pen.Dispose()
}

$g.DrawString("DZZH 载板连接框图 — 天猛星 MSPM0G3507 + 全模块（逻辑网络）", $fTitle, $dark, 40, 16)
$g.DrawString("所有外设均为插件模块。引脚名为 MCU 逻辑名；实物排针顺序落板前用万用表逐脚核对。", $fS, (New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(85,85,85))), 42, 62)

# ---------- center ----------
$TX0 = 740; $TX1 = 1300; $TY0 = 110; $TY1 = 1000
Box $TX0 $TY0 $TX1 $TY1 "天猛星 MSPM0G3507（插件）" @"
电源：5V(经 D2 输入) / GND；3V3 不接（板载输出）

I2C0 灰度      PA0 SDA / PA1 SCL
MLX 软 I2C     PB17 SCL / PB18 SDA
I2C1 授时      PA30 SDA / PA29 SCL

UART1 IMU      PA8 TX  / PA9 RX
UART0 语音     PA10 TX
UART3 LORA     PB2 TX  / PB3 RX
UART2 预留     PB15 TX / PB16 RX

PWM            PA12 A / PA13 B / PA17 C / PA7 D
方向 A         PB19 AIN1 / PB20 AIN2
方向 B         PB4 BIN1  / PB5 BIN2
方向 C         PB6 AIN1  / PB7 AIN2
方向 D         PB12 BIN1 / PB13 BIN2
STBY           PB0（两片并联）

编码器 A       PA21 A / PA26 B
编码器 B       PA31 A / PA28 B
编码器 C       PA24 A / PA25 B
编码器 D       PA14 A / PA15 B

LCD            PB9 SCLK / PB8 MOSI / PB10 RES
               PB11 DC / PB14 CS / PB26 BLK
电池检测       PA27（1/4 分压）
按键 S2        PB21（内部上拉，按下接地）
SWD            PA19 SWDIO / PA20 SWCLK
"@ "#7b3f00" "#fff8ef"

# ---------- left ----------
$LX0 = 40; $LX1 = 680
Box $LX0 110 $LX1 360 "电源（12V → 3.3V / 5V）" @"
J1 12V 电池 + → F1(5A) → D1 防反接 → VM_12V
TVS1 SMBJ15A：VM_12V ↔ GND
U1 12V→3.3V 3A  → +3V3（逻辑/模块）
U2 12V→5V  2A  → +5V（ASRPRO）
U3 5V→3.3V 1A  → +3V3_T（授时独立）
D2 SS34：+5V → TMX_5V（给天猛星，防倒灌）
C1 100uF + C2 100nF 就近放 TB6612 VM
R1 30k / R2 10k 分压 → PA27，C6 100nF
R8 1k + LED1：+3V3 电源指示
"@ "#b00020" "#fff0f0"
Arrow $LX1 235 $TX0 235 "#b00020" "+3V3/+5V"

Box $LX0 390 $LX1 540 "TB6612 #1（A 左前 / B 右前）" @"
VM→VM_12V   VCC→+3V3   GND→GND   STBY→PB0
PWMA→PA12  AIN1→PB19  AIN2→PB20
PWMB→PA13  BIN1→PB4   BIN2→PB5
AO1/AO2 → A 电机    BO1/BO2 → B 电机
"@ "#0b5394" "#eef5ff"
Arrow $TX0 465 $LX1 465 "#0b5394" "PWM+DIR"

Box $LX0 570 $LX1 720 "TB6612 #2（C 右后 / D 左后）" @"
VM→VM_12V   VCC→+3V3   GND→GND   STBY→PB0
PWMA→PA17  AIN1→PB6   AIN2→PB7
PWMB→PA7   BIN1→PB12  BIN2→PB13
AO1/AO2 → C 电机    BO1/BO2 → D 电机
"@ "#0b5394" "#eef5ff"
Arrow $TX0 645 $LX1 645 "#0b5394" "PWM+DIR"

Box $LX0 750 $LX1 930 "编码器 ×4（5P：VCC/GND/A/B/Z）" @"
A 左前：A→PA21   B→PA26   (+3V3/GND)
B 右前：A→PA31   B→PA28   (+3V3/GND)
C 右后：A→PA24   B→PA25   (+3V3/GND)
D 左后：A→PA14   B→PA15   (+3V3/GND)
开漏编码器：A/B 各 10k 上拉到 +3V3
"@ "#38761d" "#f1f8ec"
Arrow $LX1 840 $TX0 840 "#38761d" "A/B ×8"

Box $LX0 960 $LX1 1100 "落板前必查" @"
PA10/PA11 = 板载 CH340 UART0
PA21/PA18/PA2/PA23 = 特殊脚
PA5/PA6 = 40MHz HFXT，勿占用
TMX 3V3 = 板载输出，勿并载板 3V3
"@ "#a00000" "#fff0f0"

# ---------- right: 8 slots ----------
$RX0 = 1400; $RX1 = 2200
function Right($y0, $y1, $title, $lines, $color, $fill, $label) {
    Box $RX0 $y0 $RX1 $y1 $title $lines $color $fill
    Arrow $TX1 ([int](($y0 + $y1) / 2)) $RX0 ([int](($y0 + $y1) / 2)) $color $label
}

Right 110 210 "循迹灰度 AuroraT8（I2C0 0x40）" @"
VCC→+3V3  GND→GND  SDA→PA0  SCL→PA1
R3/R4 4.7k 上拉到 +3V3（载板必需）
"@ "#674ea7" "#f4f0ff" "I2C0"

Right 215 315 "MLX90614（软件 I2C 0x5A）" @"
VIN→+3V3  GND→GND  SCL→PB17  SDA→PB18
R7 4.7k 上拉 SDA（SCL 由 MCU 推挽）
"@ "#a64d79" "#fff0f7" "软 I2C"

Right 320 420 "IMU（YbIMU / JY61P，UART1）" @"
VCC→+3V3  GND→GND
模块 TX → MCU PA9 (RX)
模块 RX ← MCU PA8 (TX)
"@ "#b45f06" "#fff5e8" "UART1"

Right 425 525 "LORA 433MHz（UART3 9600）" @"
VCC→+3V3  GND→GND
模块 TXD → MCU PB3 (RX)
模块 RXD ← MCU PB2 (TX，printf 出口)
"@ "#b45f06" "#fff5e8" "UART3"

Right 530 630 "授时 ESP32-S2（I2C1 0x42）" @"
3V3→+3V3_T（独立 1A LDO）  GND→GND
SDA→PA30  SCL→PA29
R5/R6 2.2k 上拉到 +3V3
"@ "#674ea7" "#f4f0ff" "I2C1"

Right 635 735 "语音 ASRPRO2.0（UART0 115200）" @"
5V→+5V（>500mA，外接喇叭）  GND→GND
模块串口1 RX ← MCU PA10 (TX)
命令：M1..M4 / D1..D4 / E（无结束符）
"@ "#b45f06" "#fff5e8" "UART0"

Right 740 840 "LCD ST7735S 128×160（SPI1 32MHz）" @"
VCC→+3V3  GND→GND
SCLK→PB9  MOSI→PB8  RES→PB10  DC→PB11
CS→PB14  BLK→PB26
"@ "#38761d" "#f1f8ec" "SPI1"

Right 845 945 "按键 / SWD / 预留 UART2" @"
S2：PB21 ↔ GND（内部上拉）
SWD：PA19 SWDIO / PA20 SWCLK / 3V3 / GND
UART2 预留：PB15 TX / PB16 RX
"@ "#666666" "#f5f5f5" "GPIO"

$gray = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(102, 102, 102))
$g.DrawString("生成：tools/gen_carrier_hw.py + tools/draw_carrier_block.ps1    详见 pcb/connection_matrix.csv / netlist.txt / BOM.csv / design_guide.md", $fS, $gray, 42, 1110)

$out = "D:\TI_Keil\M0_SDK\mspm0_sdk_2_02_00_05\DZZH_Project\pcb\carrier_block_diagram.png"
$bmp.Save($out, [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
Write-Output "saved $out"
