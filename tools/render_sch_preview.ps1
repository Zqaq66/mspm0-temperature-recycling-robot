param(
    [double]$Scale = 3.0,
    [string]$Out = "D:\TI_Keil\M0_SDK\mspm0_sdk_2_02_00_05\DZZH_Project\pcb\DZZH_carrier_sch_preview.png",
    [double]$X0 = 0, [double]$Y0 = 0,
    [double]$X1 = 1189, [double]$Y1 = 841
)
Add-Type -AssemblyName System.Drawing

$jsonPath = "D:\TI_Keil\M0_SDK\mspm0_sdk_2_02_00_05\DZZH_Project\pcb\_sch_preview.json"
$data = Get-Content -LiteralPath $jsonPath -Raw -Encoding UTF8 | ConvertFrom-Json

$PW = $X1 - $X0; $PH = $Y1 - $Y0
$W = [int]($PW * $Scale); $H = [int]($PH * $Scale)
$bmp = New-Object System.Drawing.Bitmap($W, $H)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$g.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::AntiAliasGridFit
$g.Clear([System.Drawing.Color]::White)

$fTitle = New-Object System.Drawing.Font("Microsoft YaHei", 9)
$fRef   = New-Object System.Drawing.Font("Microsoft YaHei", 8)
$fLbl   = New-Object System.Drawing.Font("Microsoft YaHei", 7)
$pen    = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(40,40,40), 1.2)
$penWire= New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(30,90,30), 1.0)
$brush  = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(20,20,20))
$brushL = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(0,70,140))
$brushR = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(150,60,0))

function X([double]$v) { return [single](($v - $X0) * $Scale) }
function Y([double]$v) { return [single](($v - $Y0) * $Scale) }
function S([double]$v) { return [single]($v * $Scale) }

# sheet border

# wires
foreach ($wr in $data.wires) {
    $g.DrawLine($penWire, (X $wr.x1), (Y $wr.y1), (X $wr.x2), (Y $wr.y2))
}

# symbols
foreach ($s in $data.symbols) {
    $g.DrawRectangle($pen, (X $s.x), (Y $s.y), (S $s.w), (S $s.h))
    $g.DrawString($s.ref, $fRef, $brush, (X ($s.x + 1)), (Y ($s.y + 0.5)))
    $g.DrawString($s.value, $fLbl, $brushR, (X ($s.x + 1)), (Y ($s.y + 4.2)))
}

# labels
foreach ($l in $data.labels) {
    $sz = $g.MeasureString($l.text, $fLbl)
    $px = X $l.x
    $py = (Y $l.y) - ($sz.Height / 2)
    if ($l.just -eq "right") { $px = $px - $sz.Width }
    $g.DrawString($l.text, $fLbl, $brushL, $px, $py)
}

$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
Write-Output "saved $Out ($W x $H)"
