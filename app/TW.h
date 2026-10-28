#ifndef __TW_H
#define __TW_H

/* 语音播报模块 (ASRPRO2.0 / 天问Block) —— MSPM0 单向发命令, 模块负责 TTS 播报。
 * 链路: UART0 PA10(TX) -> ASRPRO 串口1 RX = PA3, 115200-8-N-1, 共地。
 * 协议/命令表见 TW.c 顶部 (命令是裸字符串, 无结束符)。
 * ★ 只发不接: 模块没接也不影响跑车。 */
void TW_Init(void);              /* 上电置 idle (UART0 外设初始化在 SYSCFG_DL_init 里) */
void Voice_RoomStart(int room);  /* 房间 1~4: 播报"请 N 号房间人员配合测温" */
void Voice_RoomDone(int room);   /* 房间 1~4: 播报"N 号房间测温完成" */
void Voice_AllDone(void);        /* 播报"本次任务完成" */

#endif
