#include "ti_msp_dl_config.h"
#include "UART.h"
#include "TW.h"

/*===========================================================================
 * 语音播报模块 (ASRPRO2.0, 天问Block 开发) —— MSPM0 只发不接
 *
 * 链路: UART0, PA10(MCU TX) -> ASRPRO **串口1** RX = PA3; 115200-8-N-1; 必须共地。
 *       (ASRPRO 串口1 TX = PA2 空着; 模块供电 3.6~5V / >500mA, 外接喇叭)
 *   ★ 用 ASRPRO 的串口1, 不要用串口0(PB5/PB6) —— 串口0 是它的下载/调试口, 会和下载冲突。
 *
 * 协议: 每条命令 = 纯 ASCII 字符串, 【不带任何结束符】
 *   M1..M4  -> 播报"请 1..4 号房间人员配合测温"
 *   D1..D4  -> 播报"1..4 号房间测温完成"
 *   E       -> 播报"本次任务完成"
 *   模块侧 (天问Block) 是 `Serial1.readString()` + `== "M1"` 的【精确比对】
 *   ★ 所以千万不要在末尾加 '\n'/'\r' —— 加了字符串就对不上, 一句都不会播。
 *     (天问的官方例程: ESP32 端 put 的是 no_newline 的裸字符串, ASRPRO 端 == 比对)
 *
 * ★ 只发不接: 模块没接/没上电, 这里只是往 UART0 写几个字节, 不影响跑车。
 * ★ 天问的"播报"块是阻塞的, 播放期间收不到新命令 -> 上层(empty.c)要等播完再发下一条。
 *===========================================================================*/

/* 逐字节发一串, 最后一个字节也等发完 (115200 下 3 字符 ~0.3ms, 可忽略) */
static void voice_send(const char *s)
{
    while ((s != 0) && (*s != 0)) UART_send_char(*s++);
    while (DL_UART_isBusy(UART0) == true);
}

/* 发 "M<n>" / "D<n>" 这种两字符命令 (无结束符 —— 见文件头说明) */
static void voice_send_room(char tag, int room)
{
    char b[3];

    if (room < 1) room = 1;
    if (room > 4) room = 4;
    b[0] = tag;
    b[1] = (char)('0' + room);
    b[2] = '\0';
    voice_send(b);
}

void TW_Init(void)
{
    /* UART0 的外设初始化在 SYSCFG_DL_init() 里, 这里没有要做的。
     * ★ 不要在这里碰 PB17/PB18: 那两个脚现在是 MLX90614 的软件 I2C (SCL/SDA),
     *   在这里拉低会把测温总线拽下去。(历史遗留: 以前这两脚曾当语音控制线用。) */
}

/* 到房间停车: 播报"请 N 号房间人员配合测温" */
void Voice_RoomStart(int room) { voice_send_room('M', room); }

/* 测完: 播报"N 号房间测温完成" */
void Voice_RoomDone(int room)  { voice_send_room('D', room); }

/* 全程收工: 播报"本次任务完成" */
void Voice_AllDone(void)       { voice_send("E"); }
