/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*===========================================================================
 * MSPM0G3507 四轮小车 —— 主程序 (main + 协作式调度器 + 两套应用层)
 *
 *   本文件: System_Init() / main() / 5·10·20·100·1000ms 任务 / 循迹 与 电机测试 两个模式
 *   两套固件二选一, 只改 [1] 里的 APP_MODE 一行; 调度器在 middle/Task.c, 时基在 middle/Timer.c
 *
 * ============================ [1] 模式与开关 ============================
 *   APP_MODE      = APP_MODE_TRACK (默认) | APP_MODE_MOTOR_TEST   ← 切固件只改这一行
 *   app_mode      = APP_MODE 的运行期副本 (各处都判它; 运行期没人改, 不是热切换)
 *   app_run       = 0 停 / 1 跑, 由板载 S2 单击切换 (上电默认 0 = 停)
 *   SKIP_OPEN_LOOP= 仅电机模式有效; 1 = 上电直接进闭环速度环 (跳过 26.5s 开环自检)
 *
 * ============================ [2] 任务划分 (middle/Task.c 调度) ============================
 *   周期     循迹模式 (TRACK)                      电机测试模式 (MOTOR_TEST)
 *   --------------------------------------------------------------------------------------
 *   5ms      Track_Run(): 读灰度+质心+PD+写PWM     —
 *   10ms     处理按键事件: 单击启停 / 长按调速      同左 (调的是闭环目标)
 *            (Key_Scan() 本身在 1ms 中断钩子里跑, 事件缓存后这里取走)
 *   20ms     编码器采样 + IMU 解析                   开环阶段表 / 四路闭环速度环, 写 PWM
 *   100ms    LCD_track() 只写文字影子(不碰 SPI)     阶段推进 + LCD_status()
 *   1000ms   串口日志 (默认整段注释掉)              同左
 *   主循环    LCD_RowFlush(): 每 1ms 最多推 1 行屏 (<=0.9ms) —— 别的模块不用等 LCD
 *   注: 5ms 槽在 IMU 转向/保向忙时改跑 ImuTurn_Update() —— 两者不会同时写 PWM
 *   注: 路口/转弯由 Mission 决定: 岔路口看路线表"要不要转"(方向由传感器判),
 *       普通弯车自己转; 转弯前先盲走 MS_ADV_MS (补传感器前伸的距离) 再原地转
 *   注: 推屏开销摊在主循环夹缝里, 不占任何任务槽, 也不会把 5ms 控制环顶掉。
 *       (改造前是 100ms 任务里整块刷 114ms, 把主循环卡死, 屏上 L 一直报 114/115)
 *   时基: TIMER_0 = 1ms 中断 -> Get_Time();  printf 出口 = LORA 口 UART3 (PB2/PB3, 9600)
 *
 * ============================ [3] 板载按键 (只用 S2 = PB21, 无需接线) ============================
 *   单击 (按住 <0.6s 松开)  ->  开始 / 停止           屏上: STOP press S2
 *   长按 (按住 >0.6s)       ->  加速一档, 之后每 0.1s 再加一档; 到上限自动回绕到下限
 *   循迹模式: 调基础速度 150~900 (默认 400)          屏上: RUN Bxxx
 *   电机模式: 调闭环目标   5~200 (默认 TARGET_SPEED) 屏上: TGT xxx CLOSED
 *   调默认值: middle/XunJi.h  TRACK_SPEED_DEFAULT / empty.c  TARGET_SPEED
 *   调手感:   middle/Key.c    KEY_LONG_TICKS(长按 60=600ms) / KEY_REPEAT_TICKS(连发 10=100ms)
 *   不想回绕: middle/XunJi.c  Track_StepSpeed() 里改成封顶 (注释里有写法)
 *
 * ============================ [4] 接线表 ============================
 *
 *   [4.1] 电源 (先接电源, 再接信号)
 *     12V 电池 +     ->  TB6612 #1 的 VM 和 #2 的 VM (两片 VM 并联)
 *     12V 电池 -     ->  MCU GND              (必须共地)
 *     LaunchPad 3V3  ->  两片 VCC             (逻辑电源必须 3.3V, 不能接 5V:
 *                                             TB6612 逻辑门限 = 0.7*VCC, 接5V时 3.5V > 3.3V 会识别不稳)
 *     LaunchPad GND  ->  两片 GND
 *     注意: 12V 绝不进 MCU / 3V3
 *
 *   [4.2] TB6612 #1 —— 前桥 (A 左前 / B 右前)
 *     TB6612 引脚        接 MCU                      接电机
 *     ------------------------------------------------------------------
 *     PWMA               PA12 (TIMG0_CCP0, 20kHz)
 *     AIN1               PB19
 *     AIN2               PB20
 *     AO1 / AO2                                     A 左前电机 + / -
 *     PWMB               PA13 (TIMG0_CCP1, 20kHz)
 *     BIN1               PB4
 *     BIN2               PB5
 *     BO1 / BO2                                     B 右前电机 + / -
 *     STBY               PB0  (两片 STBY 拧在一起接 PB0)
 *     VM / VCC / GND     见 [4.1]
 *     注: 芯片丝印的 A/B 通道 不是 轮位 A/B; #1 片的 A 通道驱动 A 轮, B 通道驱动 B 轮
 *
 *   [4.3] TB6612 #2 —— 后桥 (C 右后 / D 左后)
 *     PWMA               PA17 (TIMG7_CCP0, 20kHz)
 *     AIN1               PB6
 *     AIN2               PB7
 *     AO1 / AO2                                     C 右后电机 + / -
 *     PWMB               PA7  (TIMG7_CCP1, 20kHz)
 *     BIN1               PB12
 *     BIN2               PB13
 *     BO1 / BO2                                     D 左后电机 + / -
 *     STBY               PB0  (和 #1 的 STBY 并联)
 *     不想用 GPIO 控 STBY: 两片 STBY 直接接 3V3 (常使能), Motor_Enable() 就失效
 *
 *   [4.4] 编码器 (每路: A相 / B相 两根信号线, 另加 VCC / GND 供电)
 *     A 左前:  A相 -> PA21    B相 -> PA26      VCC -> 3V3, GND -> GND
 *     B 右前:  A相 -> PA31    B相 -> PA28      VCC -> 3V3, GND -> GND
 *     C 右后:  A相 -> PA24    B相 -> PA25      VCC -> 3V3, GND -> GND
 *     D 左后:  A相 -> PA14    B相 -> PA15      VCC -> 3V3, GND -> GND
 *     - 编码器电源用 3V3 (模块若必须 5V, A/B 相要分压或加电平转换)
 *     - 这 8 个脚是 RESISTOR_NONE (无内部上下拉); 编码器若为开漏输出, A/B 各加 10k 上拉到 3V3
 *     - A/B 相接反不用拆线: 改 app/Encoder.c 里的 ENC_x_REVERSE
 *     - 电机线 M+ / M- 接 TB6612 的 AO1/AO2 (或 BO1/BO2), 别接到编码器的信号线上
 *
 *   [4.5] 灰度传感器 (循迹模式用; 芯流 AuroraT8 八路)
 *     SDA -> PA0    SCL -> PA1    (硬件 I2C0, 400kHz, 模块地址 0x40)
 *     VCC -> 3V3    GND -> 共地    (用 3V3 可避免 I2C 上拉被抬到 5V)
 *     - 与 MLX90614 (0x5A) 共用同一条 I2C0 总线
 *     - 探头朝下, 8 路垂直于前进方向横排, 居中安装
 *
 *   [4.6] LCD (ST7735S 128x160, 硬件 SPI1 = PB9/PB8, 40MHz)
 *     SCLK -> PB9     SDA/MOSI -> PB8     RES -> PB10
 *     DC   -> PB11    CS -> PB14          BLK -> PB26      VCC -> 3V3, GND -> 共地
 *     - 刷屏走批量通道 (hardware/lcd.c 的 LCD_ShowRowFast / LCD_FillFast): CS 只翻一次,
 *       一行 4096 字节 0.82ms, 整屏填充 8.2ms; 文字层 LCD_RowSet/LCD_RowFlush 非阻塞
 *       (写影子 + 主循环 1ms 一行), 所以 LCD 不会卡住循迹/转向/以后的测温语音模块
 *
 *   [4.7] IMU (YbIMU 六轴, 姿态/转向用)
 *     VCC -> 3V3     GND -> 共地
 *     TX  -> PA8     RX  -> PA9     (模块 TX 接 MCU RX, 交叉接)
 *     - 用 UART1, 115200-8-N-1, 25Hz 自动上报 (0x7E 0x23 帧); RX 内部上拉已开
 *     - 驱动 app/imu_uart_driver.c/.h (照搬 Competition_PJ/BSP/IMU, 只把 delay.h 换成 Time.h)
 *     - 屏上 IMU 行显示当前航向 yaw (灰字 = IMU 没数据: 没接好 / TX-RX 没交叉 / 波特率不对)
 *       "RXn" (每 100ms 收到的字节数, 正常约 70~80) 要 LCD_DEBUG=1 才显示
 *
 *   [4.8] 上电前检查
 *     1) 万用表量 VM-GND / VCC-GND 不短路; VCC 实测 3.3V, VM 实测 12V
 *     2) 电池负 与 LaunchPad GND 导通 (共地)
 *     3) 四个轮子悬空再上电
 *
 * ============================ [5] 循迹模式 (APP_MODE_TRACK) ============================
 *   读法:   5ms 读 0x09 八路模拟量 (0~255, 除8数据, 越大越黑) —— 不读 0x0C 数字量
 *   算法:   平方加权质心  pos = Σ(v*v*i*100) / Σ(v*v)   -> 0~700, 中心 350 (亚路分辨率)
 *             error = pos - 350                          (负 = 线偏左)
 *           三区增益调度 PD: |err|<40 用直线软增益, >=160 用弯道激进增益, 中间线性过渡
 *             turn = (Kp*error + Kd*低通Δerror) / 1024   (Q10 定点)
 *           弯道自适应降速: 弯得越深速度越低 (最多降到 50%)
 *           左 = speed + turn, 右 = speed - turn          (turn>0 = 线偏右 -> 左轮快 = 右转)
 *           左 = A 左前 + D 左后 ; 右 = B 右前 + C 右后   (Motor_SetLeftPWM/SetRightPWM)
 *   丢线:   丢线瞬间按"丢线前最后一帧的 |error|"分流 (阈值 LOST_SIDE_ERR = 120):
 *             A 偏出 (|E| >= 120): 线被甩到一侧 -> 直接按偏向差速转弯找回, 不直行、不原地转
 *                                   (内侧轮慢 + 外侧轮快, 两轮同向);
 *                                   外侧 = LOST_TURN_SPEED(250), 差速 = LOST_TURN_DIFF(150);
 *                                   找超过 LOST_FIND_TIMEOUT_MS(2500ms) 仍未见到线 -> 停车
 *             B 中间消失 (|E| < 120): 线在中间突然没了 -> 保持直行 LOST_STRAIGHT_MS(500ms) -> 停车
 *             丢线头 60ms (LOST_KEEP_MS) 先保持上一拍输出 (防单帧误丢乱打);
 *             线一回来立刻自动恢复巡线 (不用按键)
 *   屏上 (工作模式 5 行, 一行 16 字符, 行距 16px 不重叠):
 *           第1行 状态 + 基速: STOP press S2 / LOST nnn(丢线时长ms) / Mxx 步名 子状态 Bxxx
 *           第2行 L左轮PWM R右轮PWM
 *           第3行 P质心(0~700, 中心350) E误差(负=线偏左)
 *           第4行 IMU 航向 yaw; 转向中变成 "Y 当前角 E 剩余角"
 *           第5行 授时时间 (模块没联上网就不显示; 总线故障红字 TIME I2C ERR)
 *           标定/查问题时把 LCD_DEBUG 改成 1, 会多出 3 行:
 *             8 路点位('#'=压线) + 岔路/普通弯判定(B±/C±) / X路口 H房间 N路口数 L主循环抖动
 *             / V降速后速度 RX IMU字节 ERR灰度I2C错误
 *             第9~10行 8 路原始值 (0~255, 越大越黑; 一行 4 路) —— 标定阈值看这个
 *   调参:   全部在 middle/XunJi.c 顶部 (质心阈值/极性/镜像/LR、PD 与过渡带、D 低通、
 *           降速比、丢线参数) —— 见 [7] 的顺序
 *   注:     循迹用的是开环 PWM; 想接编码器速度环, 把 Motor_SetLeft/RightPWM() 换成写
 *           Target_A/B/C/D 即可 (当前没接, 麦轮打滑会让速度环失真)
 *
 * ============================ [6] 电机测试模式 (APP_MODE_MOTOR_TEST) ============================
 *   单击 S2 开始后自动跑 (开环全程约 26.5 秒), 屏第 1 行 = 阶段 + 名字, 第 6 行 = OPEN/CLOSED:
 *     1 - 4    只有 A 左前: 正转 -> 停 -> 反转 -> 停            (1.2s / 0.8s)
 *     5 - 8    只有 B 右前: 同上
 *     9 - 12   只有 C 右后: 同上
 *     13 - 16  只有 D 左后: 同上
 *     17 - 20  四轮一起: 前进 -> 停 -> 后退 -> 停               (2s / 1s)
 *     21 - 22  四轮原地左转 / 右转                           (1.5s)
 *     23       全停                                         (1.5s)
 *     24+      闭环速度环: 四轮一直转, 第 6 行变 TGT 40 CLOSED
 *             (SKIP_OPEN_LOOP=1 时跳过 1~23 段, 一上电就是这个状态)
 *
 *   看现象定位问题:
 *     某个阶段动了别的轮子          -> 通道 / 接线错位
 *     标 FWD 却反转                 -> app/Motor.c 里 MOTOR_x_REVERSE 改 1
 *     闭环阶段 SPD 为负 / 一顿一顿  -> app/Encoder.c 里 ENC_x_REVERSE 改 1
 *     某轮 SPD 恒为 0               -> 该路编码器没接 / 相线接错
 *
 *   调参入口: empty.c 的 TARGET_SPEED 和 steps[] 表; middle/PID.c 的 Velocity_Kp/Ki/Kd
 *
 * ============================ [7] IMU 转向 / 保向 API (middle/ImuTurn.c) ============================
 *   两个动作, 都靠 yaw 闭环, 都【独占电机】—— 忙的时候循迹自动让出 (见 [2] 的注):
 *     ImuTurn_TurnDeg(delta_deg, speed)   原地差速转到指定角度, 到位自动停车
 *                                         delta_deg 正=左转(逆时针); speed 旋转 PWM (0 -> 300)
 *     ImuTurn_HoldAbs(abs_yaw, speed)     保持绝对航向: speed>0 边走边修, speed=0 原地锁角
 *   辅助:
 *     ImuTurn_IsBusy()        1 = 忙 (5ms/20ms 任务据此让出电机)
 *     ImuTurn_JustFinished()  本拍刚结束 (读后清零); ImuTurn_GetDoneReason() 看原因
 *     ImuTurn_Stop()          急停/放弃;  ImuTurn_GetYaw/GetDiff/GetTarget()  调试
 *     ImuTurn_SetDoneCallback(cb)         完成回调, 在里面做下一步动作
 *     ImuTurn_GetLastErr()    上次落点误差(度, 绝对值)  <- 验收就看它
 *     ImuTurn_GetDecel()      当前标定出的减速能力 a (度/秒^2)
 *     ImuTurn_GetCoast()      本拍预测的滑行角(度); ImuTurn_GetPulseCount() 上次用了几拍脉冲
 *     ImuTurn_ResetCalib()    标定值退回种子值 (让它重新学)
 *   结束原因: ARRIVED(到位) / STOPPED(被停) / IMU_OFF(掉线自动停) / TIMEOUT(超时自动停) / ERRWAY(方向反/被顶住)
 *   用法示例:
 *     if (ImuTurn_TurnDeg(+90, 0) == 0) { }          // 左转 90 度 (0 = 用 IMU_TURN_SPEED_MAX), 5ms 任务会自己推
 *     if (ImuTurn_JustFinished()) { 下一步; }         // 轮询完成 (或在回调里做)
 *     ImuTurn_HoldAbs(90.0f, 250);                   // 保持 90 度航向往前走
 *   调参: middle/ImuTurn.h 顶部 —— 转向主要调 IMU_TURN_SPEED_MAX (远段速度) /
 *         IMU_TURN_CUT_MARGIN (老冲过头就 +1) / IMU_TURN_ARRIVE_DEG (想到位多准)。
 *         ★ 减速能力 a 不用手调: 每次转完自动标定, 串口 TURN done 那行会打出来。
 *   看现象: 屏第 4 行平时是 "YAW +xx"(当前航向), 转向中变成 "Y +xx E +xx"(当前角/剩余角)
 *         ★ 调转向参数要看相位和角速度: 把 LCD_DEBUG 改成 1, 转向中会显示 "Y… E… 相位 角速度"
 *           相位: D=远段定速 C=近段低速 B=断电等停稳 P=脉冲微调
 *   演示: 把本文件顶部的 IMU_TURN_DEMO 改成 1 -> 单击 S2 后自动跑
 *         "原地左转 90 -> 保持该航向直行 2s -> 停车"
 *
 * ============================ [8] 调参顺序 (按这个顺序来, 一次只改一个) ============================
 *
 *   0) 先定方向 —— 还没到调 PID 这步:
 *        某轮转向与指令相反             -> app/Motor.c     MOTOR_A/B/C/D_REVERSE 改 1
 *        某轮实测速度为负 / 闭环抖       -> app/Encoder.c   ENC_A/B/C/D_REVERSE 改 1
 *        灰度探头左右装反(挡左边右边亮)   -> middle/XunJi.c  TRACK_SENSOR_FLIP 改 1
 *        越偏越往错方向打(纠正方向反)     -> middle/XunJi.c  TRACK_LR_SWAP 改 1
 *        白底数值反而比黑线大(极性反)     -> middle/XunJi.c  TRACK_DARK_IS_HIGH 改 0
 *
 *   1) 传感器自检 (不用按键: 停着也读传感器, 见下方"观察模式"): 先把 LCD_DEBUG 改成 1
 *        阈值是自动的: 上电采 20 帧白底 -> 每路一个阈值 (白底基线+60), 见 XunJi.c 顶部;
 *        白底上哪一路数值偏高 (最外侧常见) 会在第9~10行看出来, 偏太多就加大 ADAPT_MARGIN
 *        屏上会多出 8 个点位符号, 拿黑线依次挡每个探头, 位置一一对应
 *        ERR 必须恒为 0 (非 0 = 灰度 I2C 没通)
 *
 *   2) 基础速度 (单击 S2 开始 / 长按加速, 屏第1行 RUN Bxxx):
 *        先在 250 ~ 400 把循迹调稳, 稳了再往上提; 提速后 Kp 一般要同步降一点
 *        (LCD_DEBUG=1 时屏上有 Vxxx = "弯道降速后"的实际速度: 过弯时它掉下来 = 降速在工作)
 *
 *   3) 转向 PD (Q10 定点, 1024 = 1.0) —— middle/XunJi.c 顶部
 *        TRACK_KP / TRACK_KD          弯道段增益 (默认 1000 / 1200)
 *        TRACK_STRAIGHT_KP / KD       直线段增益 (默认 500 / 600, 用来压直线微振)
 *        TRANSITION_LOW / HIGH        过渡带 (默认 40 / 160), 中间线性插值, 不分档跳变
 *        D_FILTER_STRENGTH            D 项低通强度 (默认 3 = 1/8)
 *        CURVE_REDUCE_PCT             弯道最大降速% (默认 50)
 *        直线画龙        -> TRACK_STRAIGHT_KP 调小 (500 -> 300)
 *        弯道切内线/出线 -> TRACK_KP 调大 (1000 -> 1300), 同时把基础速度降一档
 *        过冲/回摆大     -> TRACK_KD 调大 (1200 -> 1800), 或把 D_FILTER_STRENGTH 加到 4
 *        过弯太慢/太急   -> CURVE_REDUCE_PCT 调小 / 调大
 *        屏第3行 P 跳变、E 不连续 -> NOISE_THRESHOLD (默认 30) 太大, 或极性反了
 *
 *   4) 丢线参数 (middle/XunJi.c 顶部):
 *        LOST_SIDE_ERR(120)        判"偏出"还是"中间消失"的 |error| 分界
 *        LOST_TURN_SPEED(250)      找回线外侧轮 PWM (别超过正常巡线速度)
 *        LOST_TURN_DIFF(150)       找回线内外差速 (内侧 = 250-150 = 100, 同向不反转)
 *        LOST_FIND_TIMEOUT_MS(2500) 偏出找回多久还没找到就停车 (填 0 = 一直找)
 *        LOST_STRAIGHT_MS(500)     中间消失后直行多久停车
 *        LOST_KEEP_MS(60)          丢线头去抖窗口 (保持上一拍输出)
 *        屏第1行 LOST nnn 是丢线时长(ms): 一直涨 = 没找回; 停在 500 附近 = 中间消失已停车;
 *        停在 2500 附近 = 偏出找超时停车
 *
 *   5) 闭环速度环 (只跑电机测试模式; SKIP_OPEN_LOOP=1 上电直接进闭环)
 *        屏上 SPD 追不上 TGT   -> TARGET 调低, 或 middle/PID.c 的 Velocity_Kp 加大 (现 5)
 *        SPD 一顿一顿 / 嗡嗡响 -> Velocity_Kp 减小, 或 Velocity_Kd 加一点 (现 0)
 *        稳态总差一点          -> Velocity_Ki 加大 (现 1); 但输出顶到 ±700 就是目标定太高
 *        SPD 符号与 TGT 反     -> app/Encoder.c 的 ENC_x_REVERSE
 *
 *   6) 调参时的两个坑:
 *        所有参数都是编译期宏 —— 改完必须重新编译下载, 没有运行时可改的入口
 *        调试日志别用 printf: 9600 下发一行要 20~40ms, 会把 5ms 循迹节拍卡住 -> 看屏幕
 *===========================================================================*/

#include <stdio.h>
#include "ti_msp_dl_config.h"
#include "Task.h"
#include "Timer.h"
#include "Time.h"
#include "lcd.h"
#include "lcd_init.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "XunJi.h"          /* 循迹层: Track_Init/Track_Run */
#include "Key.h"            /* 板载按键: 速度在线可调 */
#include "imu_uart_driver.h"/* IMU: YbIMU UART 驱动 (UART1 PA8=TX/PA9=RX) */
#include "ImuTurn.h"        /* IMU 转向/保向: 转到指定角度 / 保持指定角度 */
#include "Mission.h"        /* 任务状态机: 循迹->路口->转向->工作->恢复循迹 (路线表在 Mission.c) */
#include "MLX90614.h"       /* 红外测温 MLX90614 (I2C0 0x5A, 与灰度共用总线) */
#include "TW.h"             /* 语音播报: Voice_0/Voice_1 -> UART0 */
#include "UART.h"           /* UART_Init / UART_send_char -> UART0 (语音) */
#include "TimeSync.h"        /* I2C 授时查询端: ESP32-S2(0x42) -> 本地走时 (接线/调参见 TimeSync.h) */

/* ---- 任务状态机总开关 ----
 * 1 = 单击 S2 后按 Mission.c 的路线表跑 (循迹+路口检测+定角转弯+步间停车 + 工作钩子)
 * 0 = 退回"纯循迹" (原来的行为, 台架调循迹/转弯参数时用)
 * ★ 台架一段一段验: 不用改这里, 改 Mission.c 顶部的 MISSION_DEBUG_STEPS = N
 *    (只跑路线表前 N 步就收工停车; 1=只验"循迹到第一个路口", 2=再验"盲走穿口", ...)
 * ★ 观察模式: 没开始时也读灰度刷屏, 只是不驱动电机 (XunJi 的 Track_SetDrive(0)),
 *    所以标定探头/路口/房间时不用按键, 手推车或拿黑线挡探头屏上就实时变 */
#define MISSION_ENABLE   1

/* ---- 题目功能: 房间测温环节 (语音 / 测温 / 电量 / 屏显结果区 / LORA 上报) ----
 * 1 = 开 (默认): Mission_OnWork() 里做完整流程; 屏第 6~10 行显示 [电量 + 4 个房间结果]
 * 0 = 关: 屏第 6~10 行还给 LCD_DEBUG 的标定/诊断行 (台架调循迹/路口时用)
 * ★ WORK_FEATURES 和 LCD_DEBUG 抢屏第 6~10 行 -> 只能二选一 */
#ifndef WORK_FEATURES
#define WORK_FEATURES   1
#endif

/* 电池电压: ADC0/PA27, 板上 1/4 分压 -> Vbat = Vpin * BAT_DIV_RATIO
 * 电量% = (Vbat - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V) * 100 (题目给的参考公式) */
#define BAT_DIV_RATIO   4.0f
#define BAT_EMPTY_V     10.5f
#define BAT_FULL_V      12.6f

/* 测温: TEMP_SAMPLES 次取中位, 两次间隔 TEMP_GAP_MS (MLX90614 内部约 10Hz 刷新) */
#define TEMP_SAMPLES    4
#define TEMP_GAP_MS     25

#define TARGET_SPEED   40       /* 闭环目标: 编码器计数 / 20ms (先给小的, 按实测调) */

/* ---- 调试开关 ----
 * 1 = 上电直接进闭环速度环 (跳过 26.5 秒开环自检, 专门调速度环用)
 * 0 = 正常流程: 先跑 23 段开环自检(验证通道/转向), 再进闭环 */
#define SKIP_OPEN_LOOP   1

/* ---- 应用模式 (二选一) ----
 * APP_MODE_MOTOR_TEST : 电机测试 (开环阶段表 -> 闭环速度环)
 * APP_MODE_TRACK      : 循迹 (5ms 读灰度 -> 加权质心 -> PD -> 左右 PWM) */
#define APP_MODE_MOTOR_TEST  0
#define APP_MODE_TRACK       1
#define APP_MODE             APP_MODE_TRACK

uint8_t app_mode = APP_MODE;

/* ---- 运行开关: S2(PB21) 单击切换 ----
 * 0 = 停 (上电默认): 电机输出全 0, 循迹不跑, 屏上显示 STOP
 * 1 = 跑: 循迹 / 电机测试按各自流程走 */
uint8_t app_run = 0;

/* IMU 转向/保向"刚结束"标志: 由 5ms 任务在 ImuTurn_JustFinished() 时置 1,
 * 应用层用完自己清 0 (完成原因看 ImuTurn_GetDoneReason()) */
uint8_t imu_turn_done = 0;

/* ---- 屏幕调试行开关 ----
 * 0 = 工作 (默认): 只显示 5 行工作信息, 屏干净
 *                   第1行 状态+基速 / 第2行 左右PWM / 第3行 质心误差 / 第4行 IMU航向 / 第5行 时间
 * 1 = 台架调试  : 额外多出 3 行 (8路点位 / X H L K / V RX ERR)
 *                   标定传感器、查路口检测、查实时性时打开; 平时关掉 */
#ifndef LCD_DEBUG          /* 也可以用 Keil 的 Misc Controls 加 -DLCD_DEBUG=1 覆盖 */
#define LCD_DEBUG       1
#endif

/*---------------------------------------------------------------------------
 * 1ms 中断钩子: 按键扫描放这里
 *   主循环是协作式的, LCD 刷一屏要几十 ms、串口 printf 一行要几十 ms,
 *   这些都会把 10ms 任务拖到几十 ms 一次 —— 按键放在任务里扫就会漏掉快速单击。
 *   放 1ms 定时器中断里扫, 主循环再怎么卡也不会漏按。
 *   事件缓存到 key_ev_pending, 由 APP_Task_10ms() 取走处理。
 *-------------------------------------------------------------------------*/
volatile int      key_ev_pending = 0;   /* 按键事件缓存 (中断里或, 10ms 任务里清零) */
volatile uint16_t key_scan_cnt   = 0;   /* 按键扫描次数累计 (诊断用: 每100ms应≈100) */
volatile uint16_t app_loop_ms    = 0;   /* 5ms 任务实际间隔 (诊断用: 正常=5, 变大=主循环被卡) */

void APP_SysTick_Hook(void)
{
    key_ev_pending |= Key_Scan();       /* 1ms 一次, 返回 0 或事件位 */
    key_scan_cnt++;
}

/* ---- IMU 转向演示开关 ----
 * 1 = 编进固件: 单击 S2 开始后自动跑一遍
 *       原地转 IMU_TURN_DEMO_DEG 度 (正=左/逆时针, 负=右/顺时针)
 *       -> 再反向转同样角度 (IMU_TURN_DEMO_BACK=1, 验证反方向 + 从非零角度起步)
 *       -> 原地锁住该角度 2s (speed=0, 车不动) -> 停车
 *       要测 180 度: 把下面 IMU_TURN_DEMO_DEG 改成 180 就行 (v7 门限是算出来的, 不用改参数)
 * 只用于台架验证 ImuTurn 的接口, 正式跑改回 0 (演示期间循迹不参与) */
#define IMU_TURN_DEMO   0        /* ★ 开了任务状态机(MISSION_ENABLE=1)就必须关掉它:
                                  * 演示和状态机会同时调 ImuTurn, 互相抢电机。
                                  * 要单独台架测转弯: 这里改 1, 同时把 MISSION_ENABLE 改 0 */
#define IMU_TURN_DEMO_DEG  180    /* 演示转多少度: 90 或 180 或 -90 或 -180 (v7 自动适应)  */
#define IMU_TURN_DEMO_BACK  1    /* 1 = 转完再【反向】转同样角度 (验证反方向 + 从非零角度起步) */

#if IMU_TURN_DEMO
static void App_ImuTurnDemo(void)
{
    static uint8_t  step = 0;
    static uint32_t t0   = 0;

    if (!app_run)                              /* 停止 -> 演示复位 */
    {
        step = 0;
        return;
    }

    switch (step)
    {
    case 0:                                    /* 起步: 原地左转 IMU_TURN_DEMO_DEG 度 */
        if (!ImuTurn_IsBusy() && (ImuTurn_TurnDeg(IMU_TURN_DEMO_DEG, 0) == 0))
        {
            printf("TURN1 start: yaw=%d a=%d\r\n", (int)ImuTurn_GetYaw(), (int)ImuTurn_GetDecel());
            step = 1;
        }
        break;

    case 1:                                    /* 第一转到位 */
        if (imu_turn_done)
        {
            imu_turn_done = 0;
            /* 台架数据: 结束原因(0=到位 3=超时 4=越走越远) / 目标 / 落点 / 落点误差 */
            printf("TURN1 done: reason=%d target=%d yaw=%d err=%d a=%d pulse=%d\r\n",
                   ImuTurn_GetDoneReason(), (int)ImuTurn_GetTarget(),
                   (int)ImuTurn_GetYaw(), (int)ImuTurn_GetDiff(),
                   (int)ImuTurn_GetDecel(), ImuTurn_GetPulseCount());
#if IMU_TURN_DEMO_BACK
            /* 再【反向】转同样角度: 这次起点是个非零角度, 用来验证"任意角度起步 + 反向" */
            if (!ImuTurn_IsBusy() && (ImuTurn_TurnDeg(-IMU_TURN_DEMO_DEG, 0) == 0))
            {
                printf("TURN2 start: yaw=%d a=%d\r\n", (int)ImuTurn_GetYaw(), (int)ImuTurn_GetDecel());
                step = 2;
            }
#else
            t0 = Get_Time();
            ImuTurn_HoldAbs(ImuTurn_GetYaw(), 0);
            step = 3;
#endif
        }
        break;

#if IMU_TURN_DEMO_BACK
    case 2:                                    /* 第二转(反向)到位 -> 原地锁角 2s */
        if (imu_turn_done)
        {
            imu_turn_done = 0;
            printf("TURN2 done: reason=%d target=%d yaw=%d err=%d a=%d pulse=%d\r\n",
                   ImuTurn_GetDoneReason(), (int)ImuTurn_GetTarget(),
                   (int)ImuTurn_GetYaw(), (int)ImuTurn_GetDiff(),
                   (int)ImuTurn_GetDecel(), ImuTurn_GetPulseCount());
            t0 = Get_Time();
            ImuTurn_HoldAbs(ImuTurn_GetYaw(), 0);   /* 0 = 原地锁角, 车轮不转 */
            step = 3;
        }
        break;
#endif

    case 3:                                    /* 保持 2s -> 停车 */
        if ((Get_Time() - t0) >= 2000)
        {
            ImuTurn_Stop();
            step = 4;
        }
        break;

    default:
        break;
    }
}
#endif

/* 开环测试步骤: {A, B, C, D, 持续时间ms, 阶段名} */
typedef struct {
    int16_t  a, b, c, d;
    uint16_t ms;
    char     name[10];
} TestStep;

static const TestStep steps[] = {
    { 300,   0,   0,   0, 1200, "A FWD"},  {   0,   0,   0,   0, 800, "STOP"},
    {-300,   0,   0,   0, 1200, "A REV"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0, 300,   0,   0, 1200, "B FWD"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0,-300,   0,   0, 1200, "B REV"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0,   0, 300,   0, 1200, "C FWD"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0,   0,-300,   0, 1200, "C REV"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0,   0,   0, 300, 1200, "D FWD"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0,   0,   0,-300, 1200, "D REV"},  {   0,   0,   0,   0, 800, "STOP"},
    { 400, 400, 400, 400, 2000, "ALL FWD"},
    {   0,   0,   0,   0, 1000, "STOP"},
    {-400,-400,-400,-400, 2000, "ALL REV"},
    {   0,   0,   0,   0, 1000, "STOP"},
    {-300, 300,-300, 300, 1500, "TURN L"},      /* 与 Motor.c 的 Turn_left() 一致 */
    { 300,-300, 300,-300, 1500, "TURN R"},      /* 与 Turn_right() 一致 */
    {   0,   0,   0,   0, 1500, "STOP"},
};
#define STEP_NUM   (sizeof(steps)/sizeof(TestStep))    /* 走完这些步就进闭环 */

 
static uint8_t  stage    = 0;      /* 当前阶段; >= STEP_NUM 表示已进闭环 */
static uint32_t stage_ms = 0;      /* 当前阶段已运行时长 */

int PWMA=0,PWMB=0,PWMC=0,PWMD=0;                  /* 4 路 PWM 指令 */
int A_speed=0,B_speed=0,C_speed=0,D_speed=0;      /* 4 路实测速度(计数/20ms) */
static int target_speed = TARGET_SPEED;           /* 闭环目标, 电机模式下按键可调 */
#define TARGET_SPEED_MIN      5
#define TARGET_SPEED_MAX    200
#define TARGET_SPEED_STEP     5

int Target_A=TARGET_SPEED,Target_B=TARGET_SPEED,Target_C=TARGET_SPEED,Target_D=TARGET_SPEED;

/*---------------------------------------------------------------------------
 * LCD 行输出 (统一入口, 防止印到屏外)
 *   屏: 128 x 160 (USE_HORIZONTAL=1), 16 号字 = 8 px 宽 -> 一行顶多 16 个字符。
 *   规则: 不足 16 字符补空格(盖掉上一帧的残字), 超过 16 字符截断(不然会印出屏外看不到)。
 *   所有 LCD 文字行都走这个函数, 别直接 LCD_ShowString。
 *   ★ 这里只把文字写进"影子"(LCD_RowSet, 纯 RAM, 几 us 返回), 一个字节 SPI 都不发;
 *     真正推屏在主循环的 LCD_RowFlush() 里 1ms 最多一行地做掉 —— 所以本函数不会卡主循环。
 *     内容/颜色没变的话 LCD_RowSet 里就判掉了, 连脏行都不标 (静止画面 0 流量)。
 *     (补空格/截断也搬进 LCD_RowSet 了; 参数 y 仍是像素, 内部 /16 换成行号)
 *-------------------------------------------------------------------------*/
#define LCD_COLS   16
static void lcd_row(int y, char *buf, unsigned fc)
{
    LCD_RowSet((u8)(y / 16), buf, (u16)fc);
}

/*---------------------------------------------------------------------------
 * 行布局 —— y 只能是 16 的倍数 (0,16,32,...,144)
 *   屏 128x160, 16 号字 = 8px 宽 x 16px 高 -> 一行 16 字符, 竖着正好 10 行。
 *   ★ 别再引入非 16 倍数的 y: 以前用过 y=136, 字高 16 -> 占到 151, 和 y=144 那行
 *     叠了 8 个像素, 屏上就是两行糊在一起 (2026-10-06 修掉)。
 *   工作模式 (LCD_DEBUG=0) 只用 R_STEP..R_TIME 这 5 行; 调试模式再加 R_BITS..R_IO。
 *-------------------------------------------------------------------------*/
#define LCD_Y(n)     ((n) * 16)          /* 行号 -> y 像素, n = 0..9 */
#define R_STEP       0                   /* 步骤/状态 + 基速 (或 STOP / LOST) */
#define R_PWM        1                   /* 左右轮 PWM */
#define R_TRACK      2                   /* 质心 + 误差 */
#define R_IMU        3                   /* IMU 航向 (转向时 + 剩余角) */
#define R_TIME       4                   /* 授时时间 (未同步时整行空) */
#define R_BITS       5                   /* [仅调试] 8 路点位 */
#define R_DIAG       6                   /* [仅调试] X/H 电平 + 主循环抖动 + 按键扫描率 */
#define R_IO         7                   /* [仅调试] 降速后速度 + IMU 字节 + 灰度I2C错误 */
#define R_RAW0       8                   /* [仅调试] 8 路原始值 ch0~3 (标定阈值时看这个) */
#define R_RAW1       9                   /* [仅调试] 8 路原始值 ch4~7 */


/*---------------------------------------------------------------------------
 * IMU 行 (工作模式占 R_IMU = 第4行)
 *   平时:   "YAW   +12"      当前航向(度), 已按 IMU_TURN_YAW_SIGN 校正并归一化
 *   转向中: "Y +12 E +78"    当前航向 + 距目标的剩余角(度)
 *   颜色:   绿 = 有数据, 灰 = IMU 没数据; 转向中黄色
 *   ★ 调转向参数时要看相位(D远段/C近段/B等停稳/P脉冲)和角速度 -> 把 LCD_DEBUG 改成 1,
 *     转向中就会显示成 "Y +12 E +78 D 113" (正好 16 字符)
 *-------------------------------------------------------------------------*/
static void LCD_imu(int row)
{
    char  buf[24];
    float euler[3];
    int   yaw = 0;
    int   ok  = (IMU_GetEuler(euler) == 0);

    if (ok)
    {
        /* 按 IMU_TURN_YAW_SIGN 校正并归一化到 (-180,180]: 屏上的 yaw 和命令同向 (+ = 左转),
         * 而且从任意角度起都不会显示成 -350 这种值。
         * 注意 IMU 原始报值的符号是相反的 (罗盘式顺时针为正), 别拿屏上的数去和上位机对 */
        float y = euler[2] * (float)IMU_TURN_YAW_SIGN;
        while (y >  180.0f) y -= 360.0f;
        while (y < -180.0f) y += 360.0f;
        yaw = (int)((y >= 0.0f) ? (y + 0.5f) : (y - 0.5f));
    }

    if (ImuTurn_IsBusy())                        /* 转向/保向中: 当前角 + 剩余角 */
    {
#if LCD_DEBUG
        int w = (int)ImuTurn_GetRate();          /* 调试时补上相位和角速度 (调转向参数用) */
        if (w >  99) w =  99;
        if (w < -99) w = -99;
        sprintf(buf, "Y%+4d E%+4d %c%3d", yaw, (int)ImuTurn_GetDiff(), ImuTurn_GetPhase(), w);
#else
        sprintf(buf, "Y%+4d E%+4d", yaw, (int)ImuTurn_GetDiff());
#endif
        lcd_row(LCD_Y(row), buf, YELLOW);
    }
    else                                         /* 平时: 只显示当前航向 */
    {
        sprintf(buf, "YAW   %+5d", yaw);
        lcd_row(LCD_Y(row), buf, ok ? GREEN : GRAY);   /* 灰 = IMU 没数据 */
    }
}


/*---------------------------------------------------------------------------
 * 授时时间行 (工作模式占 R_TIME = 第5行)
 *   st ==  1: 绿字 "10-06 14:05:23" (14 字符) —— 秒是本板自己走出来的, 不用每秒读总线
 *   st ==  0: 整行空 (模块还没联上网; 上电 3~5s 内属正常, 不报警)
 *   st == -1: 红字 "TIME I2C ERR" (总线故障: 查线/上拉/供电)
 *   ts_last_status: 给 1000ms 任务判断"状态变了没", 只在变化时打一行串口
 *-------------------------------------------------------------------------*/
static int ts_last_status = 99;     /* 授时状态上报用: 1000ms 任务里看它变没变 */

static void LCD_clock(int row)
{
    char buf[24];
    int  st = TimeSync_Status();

    if (st == 1)
    {
        sprintf(buf, "%s", TimeSync_GetClock());
        lcd_row(LCD_Y(row), buf, GREEN);
    }
    else if (st == 0)
    {
        buf[0] = 0;                                  /* 按要求: 未同步不显示 */
        lcd_row(LCD_Y(row), buf, GRAY);
    }
    else
    {
        sprintf(buf, "TIME I2C ERR");                /* 总线故障: 查线/上拉/供电 */
        lcd_row(LCD_Y(row), buf, RED);
    }
}


/*===========================================================================
 * 题目功能: 房间测温环节 (MS_WORK 步里由 Mission_OnWork() 调)
 *   流程: 语音提示 -> 测温 -> 读电量 -> 记结果(屏+时间戳) -> LORA 上报 -> 语音完成
 *   结果存 g_room[]: 中途不清, 下一次任务开始(Results_Clear)才清 (题目要求)
 *   ★ 本工程用 microlib, printf/sprintf 不支持 %f -> 浮点一律走 fmt_temp/fmt_volt 整数化
 *===========================================================================*/
#if WORK_FEATURES
typedef struct
{
    uint8_t valid;          /* 1 = 这个房间已经测过 */
    uint8_t time_ok;        /* 1 = 测量时刻有效 (I2C 授时已同步) */
    float   temp_c;         /* 体温 ℃ */
    float   bat_v;          /* 测量时的电池电压 V */
    uint8_t bat_pct;        /* 测量时的电量 0~100 */
    uint8_t hh, mm, ss;     /* 测量时刻 时:分:秒 */
} RoomResult;

static RoomResult g_room[5];          /* 只用 1..4 (下标 = 房间号) */
static float      g_bat_v   = 0.0f;   /* 最近一次读到的电池电压 (结果区第 1 行显示) */
static uint8_t    g_bat_pct = 0;
static uint8_t    g_task_done = 0;    /* 1 = 整表跑完 (屏第 1 行提示"任务完成") */

/* 清空结果 —— 在下一次任务开始时调 (满足"数据中途不得清除") */
static void Results_Clear(void)
{
    int i;
    for (i = 0; i < 5; i++) { g_room[i].valid = 0; g_room[i].time_ok = 0; }
    g_bat_v     = 0.0f;
    g_bat_pct   = 0;
    g_task_done = 0;
}

/* 读一次电池电压: ADC0/PA27 (SysConfig 已配: 12bit / 重复采样 / 软件触发)
 *   Vpin = raw/4095*3.3V;  板上 1/4 分压 -> Vbat = Vpin * BAT_DIV_RATIO */
static float Bat_ReadVoltage(void)
{
    uint16_t raw;
    uint32_t t0;

    DL_ADC12_startConversion(ADC_VOLTAGE_INST);
    t0 = Get_Time();
    while (!(DL_ADC12_getRawInterruptStatus(ADC_VOLTAGE_INST,
                                            DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED)))
    {
        if ((Get_Time() - t0) > 20u) break;      /* 兜底: 硬件没接也别把 5ms 任务卡死 */
    }
    DL_ADC12_clearInterruptStatus(ADC_VOLTAGE_INST,
                                  DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);

    raw = DL_ADC12_getMemResult(ADC_VOLTAGE_INST, ADC_VOLTAGE_ADCMEM_ADC_CH0);
    return ((float)raw * 3.3f / 4095.0f) * BAT_DIV_RATIO;
}

/* 连续读 4 次求平均 (去抖) */
static float Bat_ReadAvg(void)
{
    float sum = 0.0f;
    int   k;
    for (k = 0; k < 4; k++) sum += Bat_ReadVoltage();
    return sum * 0.25f;
}

static uint8_t Bat_ToPercent(float v)
{
    float pct = (v - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V) * 100.0f;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8_t)(pct + 0.5f);
}

/* 读体温: MLX90614 物体温度 (寄存器 0x07), 采 TEMP_SAMPLES 次取中位数
 *   驱动读失败返回 0.0f (见 MLX90614.c), 这里把 0.0 和超范围值都当失败丢掉 */
static float Temp_ReadBody(uint8_t *ok)
{
    float s[TEMP_SAMPLES];
    int   n = 0, i, j;

    for (i = 0; i < TEMP_SAMPLES; i++)
    {
        float t = MLX90614_Read(0x5A, 0x07);
        if ((t != 0.0f) && (t > -10.0f) && (t < 80.0f)) s[n++] = t;
        delay_ms(TEMP_GAP_MS);
    }
    *ok = (n > 0) ? 1 : 0;
    if (n == 0) return 0.0f;

    for (i = 1; i < n; i++)                      /* 插入排序 (n <= TEMP_SAMPLES) */
    {
        float key = s[i];
        for (j = i - 1; (j >= 0) && (s[j] > key); j--) s[j + 1] = s[j];
        s[j + 1] = key;
    }
    return (n & 1) ? s[n / 2] : (s[n / 2 - 1] + s[n / 2]) * 0.5f;
}

/* 浮点 -> 整数格式化 (microlib 无 %f): 温度 "36.5" / 电压 "11.85V" */
static void fmt_temp(char *out, float t)
{
    int t10 = (int)(t * 10.0f + ((t >= 0.0f) ? 0.5f : -0.5f));
    if (t10 < 0) sprintf(out, "-%d.%d", (-t10) / 10, (-t10) % 10);
    else         sprintf(out, "%d.%d", t10 / 10, t10 % 10);
}
static void fmt_volt(char *out, float v)
{
    int mv = (int)(v * 100.0f + 0.5f);
    if (mv < 0) mv = 0;
    sprintf(out, "%d.%02dV", mv / 100, mv % 100);
}

/* 房间结果区: 第 6 行 = 电量; 第 7~10 行 = 4 个房间 (题目要求: 中途不清) */
static void LCD_results(void)
{
    char buf[24];
    char tmp[10];
    int  r;

    if (g_bat_v > 1.0f) { fmt_volt(tmp, g_bat_v); sprintf(buf, "BAT %s %d%%", tmp, (int)g_bat_pct); }
    else                sprintf(buf, "BAT  --.--V ---");
    lcd_row(LCD_Y(5), buf, (g_bat_pct && (g_bat_pct < 20)) ? RED : GREEN);

    for (r = 1; r <= 4; r++)
    {
        if (g_room[r].valid)
        {
            fmt_temp(tmp, g_room[r].temp_c);
            if (g_room[r].time_ok)
                sprintf(buf, "%d %sC %02d:%02d:%02d", r, tmp,
                        (int)g_room[r].hh, (int)g_room[r].mm, (int)g_room[r].ss);
            else
                sprintf(buf, "%d %sC --:--:--", r, tmp);
            lcd_row(LCD_Y(5 + r), buf, YELLOW);
        }
        else
        {
            sprintf(buf, "%d --.-C --:--:--", r);
            lcd_row(LCD_Y(5 + r), buf, GRAY);
        }
    }
}
#endif /* WORK_FEATURES */

/*---------------------------------------------------------------------------
 * 电机测试模式 (APP_MODE_MOTOR_TEST) 的 LCD —— 台架验证用, 8 行
 *   第1行 阶段+名字 / 第2-3行 PWM A~D / 第4-5行 实测速度 A~D /
 *   第6行 闭环目标 OPEN|CLOSED / 第7行 IMU 航向 / 第8行 授时时间
 *   只写文字影子 (LCD_RowSet), 不碰 SPI —— 推屏在主循环 LCD_RowFlush() 里 1ms/行做
 *-------------------------------------------------------------------------*/
static void LCD_status(void)
{
    char buf[24];

    if (!app_run)               sprintf(buf, "STOP  press S2 ");
    else if (stage >= STEP_NUM) sprintf(buf, "CLOSED LOOP    ");
    else                        sprintf(buf, "%2d %-11s", stage + 1, steps[stage].name);
    lcd_row(LCD_Y(0), buf, app_run ? YELLOW : RED);

    sprintf(buf, "PWM A%4d B%4d", PWMA, PWMB);     /* 15 字符 */
    lcd_row(LCD_Y(1), buf, WHITE);

    sprintf(buf, "PWM C%4d D%4d", PWMC, PWMD);     /* 15 字符 */
    lcd_row(LCD_Y(2), buf, WHITE);

    sprintf(buf, "SPD A%4d B%4d", A_speed, B_speed);  /* 15 字符 */
    lcd_row(LCD_Y(3), buf, GREEN);

    sprintf(buf, "SPD C%4d D%4d", C_speed, D_speed);  /* 15 字符 */
    lcd_row(LCD_Y(4), buf, GREEN);

    if (stage >= STEP_NUM) sprintf(buf, "TGT %4d CLOSED", Target_A);
    else                   sprintf(buf, "TGT ---- OPEN  ");
    lcd_row(LCD_Y(5), buf, CYAN);

    LCD_imu(6);                              /* 第7行 */
    LCD_clock(7);                            /* 第8行 */
}


/*---------------------------------------------------------------------------
 * 循迹模式 (APP_MODE_TRACK) 的 LCD —— 工作模式 5 行, 不重叠/不超屏
 *   R_STEP  第1行 状态 + 基速: STOP press S2 / LOST nnn(丢线) / Mxx 步名 子状态 Bxxx
 *   R_PWM   第2行 左右轮 PWM
 *   R_TRACK 第3行 P 质心(0~700, 中心350) E 误差(负 = 线偏左)
 *   R_IMU   第4行 IMU 航向 (转向中 = 当前角 + 剩余角)
 *   R_TIME  第5行 授时时间 (未同步时不显示)
 *   LCD_DEBUG=1 再补 3 行: 8路点位 / X H L K / V RX ERR (标定和查问题时用)
 *-------------------------------------------------------------------------*/
static void LCD_track(void)
{
    char buf[24];

    /* 第1行: 状态 —— 停 / 丢线 / 状态机步号+子状态 / 纯循迹, 后两种都带基速 B */
#if MISSION_ENABLE
    /* 任务停下来报警: fail=1 找线失败(NOLINE) / fail=2 转弯序列超时或 IMU 起不来 (参见 Mission.c) */
    if (!app_run && Mission_GetFail() == 2)     sprintf(buf, "TURN  fail  ");
    else if (!app_run && Mission_GetFail())     sprintf(buf, "NOLINE  stop  ");
#if WORK_FEATURES
    else if (g_task_done)                       sprintf(buf, "TASK  DONE  ");
#endif
    else
#endif
    if (!app_run)                sprintf(buf, "STOP  press S2  ");
    else if (track_lost)         sprintf(buf, "LOST %-5d     ", track_lost);
#if MISSION_ENABLE
    else if (Mission_IsActive()) sprintf(buf, "%s B%3d", Mission_GetStatus(), Track_GetBaseSpeed());
#endif
    else                         sprintf(buf, "RUN    B%3d  ", Track_GetBaseSpeed());
    lcd_row(LCD_Y(R_STEP), buf, app_run ? CYAN : RED);

    /* 第2行: 左右轮 PWM */
    sprintf(buf, "L%+4d R%+4d", track_left_pwm, track_right_pwm);
    lcd_row(LCD_Y(R_PWM), buf, WHITE);

    /* 第3行: 质心 + 误差 */
    sprintf(buf, "P%3d E%+4d", track_pos, track_error);
    lcd_row(LCD_Y(R_TRACK), buf, YELLOW);

    LCD_imu(R_IMU);                          /* 第4行: 航向 (转向中 + 剩余角) */
    LCD_clock(R_TIME);                       /* 第5行: 授时时间 (未同步不显示) */

#if WORK_FEATURES
    LCD_results();                           /* 第 6~10 行: 电量 + 4 个房间结果 (题目要求) */
#endif

#if LCD_DEBUG && !WORK_FEATURES
    {   /* 以下 3 行只在 LCD_DEBUG=1 时显示: 标定传感器 / 查路口检测 / 查实时性 */
        static uint16_t loop_max = 0;
        static uint32_t last_rx  = 0;
        uint32_t rx = IMU_GetRxByteCount();
        unsigned rx_delta = (unsigned)(rx - last_rx);
        int      i;

        last_rx = rx;

        for (i = 0; i < 8; i++) buf[i] = ((track_bits >> i) & 0x01) ? '.' : '#';
        /* 后 4 格 = 岔路/普通弯 判定结果 (标定 XunJi.c 的 BRANCH_WIN_FRAMES 就看这个):
         *   B+ = 判成岔路, 分支在右   B- = 岔路在左 (主线还在, 可以直行/可以拐)
         *   C+ = 判成普通弯, 往右拐   C- = 普通弯往左 (主线自己拐走了 -> 车自己转)
         *   SUP = 现在判定被关着 (转向过程 / 转完后的屏蔽窗口)   --- = 没有判定结果 */
        if (Track_GetSuppressed()) sprintf(buf + 8, " SUP");   /* 判定被关着 (转向中/收工后的屏蔽窗口) */
        else if (track_branch)     sprintf(buf + 8, " B%c", (track_branch_side > 0) ? '+' : '-');
        else if (track_corner)     sprintf(buf + 8, " C%c", (track_corner_side > 0) ? '+' : '-');
        else                       sprintf(buf + 8, " ---");
        lcd_row(LCD_Y(R_BITS), buf, GREEN);

        if (app_loop_ms > loop_max) loop_max = app_loop_ms;    /* 5ms 任务最坏间隔 (正常 5) */
        /* X = 路口电平(最左或最右压线)  H = 房间黑块电平  N = 上电以来路口累计数
         * T = 上一次转弯"看到线时"线偏阵列中心多少 (x10 路, 正 = 偏右)
         * F = 上一次转弯靠哪一步找到线: 0 = 转到位(没找) 1 = 直进 2 = 弧A 3 = 弧B 4 = 直退 5 = 扫描 9 = 失败
         *   ★ 标定转弯落点看 T/F: T 一直很大说明前移/转角速度要调; F 经常 >=2 说明落点已经偏了。
         * (原来这行还有 L = 循环最坏间隔 / K = 按键扫描率, 都撤掉腾地方给转弯标定) */
        sprintf(buf, "X%d H%d N%d T%+d F%d", track_cross, track_room, track_cross_cnt,
                Mission_GetTurnDev(), Mission_GetTurnFound());
        lcd_row(LCD_Y(R_DIAG), buf, track_cross ? YELLOW : GRAY);
        loop_max = 0;

        if (rx_delta > 999) rx_delta = 999;
        sprintf(buf, "V%3d RX%3u E%2d", track_speed, rx_delta, track_i2c_err);
        lcd_row(LCD_Y(R_IO), buf, track_i2c_err ? RED : GRAY);

        /* 8 路原始值 (0~255, 越大越黑) —— 标定阈值就看这两行:
         * 白底上哪几路本来就高 (常见是最外侧) 一眼能看出来, 阈值 = 白底值 + ADAPT_MARGIN(60)。
         * 对照上面 R_BITS 那行: 数值明显高的那几路应该正好是 '#'。上电标定完串口会打一行基线+阈值。 */
        sprintf(buf, "%3u %3u %3u %3u", (unsigned)track_raw[0], (unsigned)track_raw[1],
                                        (unsigned)track_raw[2], (unsigned)track_raw[3]);
        lcd_row(LCD_Y(R_RAW0), buf, (track_raw[0] > track_thr[0] || track_raw[3] > track_thr[3]) ? YELLOW : GRAY);

        sprintf(buf, "%3u %3u %3u %3u", (unsigned)track_raw[4], (unsigned)track_raw[5],
                                        (unsigned)track_raw[6], (unsigned)track_raw[7]);
        lcd_row(LCD_Y(R_RAW1), buf, (track_raw[4] > track_thr[4] || track_raw[7] > track_thr[7]) ? YELLOW : GRAY);
    }
#endif
}


/*---------------------------------------------------------------------------
 * 5ms 任务: 循迹 (读灰度 + 控制)
 *-------------------------------------------------------------------------*/
void APP_Task_5ms(void)
{
    /* 诊断: 本任务的实际调用间隔 (= 主循环一圈的时间)。正常 = 5ms */
    {
        static uint32_t last_ms = 0;
        uint32_t now_ms = Get_Time();
        app_loop_ms = (uint16_t)(now_ms - last_ms);
        last_ms     = now_ms;
    }

    /* IMU 转向/保向期间独占电机: 循迹让出, 由 ImuTurn 推进 */
    if (ImuTurn_IsBusy())
    {
        ImuTurn_Update();

        if (ImuTurn_JustFinished())          /* 刚结束: 循迹清状态, 下一拍自然接管 */
        {
            Track_Reset();
            imu_turn_done = 1;
        }
        return;
    }

    /* 驱动闸门: 跟着 app_run 走 —— 没开始时也照读灰度刷屏, 只是不驱动电机 (台架标定用:
     * 手推车 / 拿黑线挡探头时, 屏上 8 路点位 / X / H / N 都实时变, 车不动)。
     * ★ 必须放在 Mission 分支【前面】: 状态机接管后也要保证 drive_en 已经打开,
     *   否则 Mission_Run() 里 MS_TRACK -> Track_Run() -> track_output() 全被闸门挡掉,
     *   现象就是"按 S2 后屏上 L/R 数字在跳、车不走"。(2026-10-06 修)
     *   开关状态变了才切, 不每拍写寄存器 */
    if (app_mode == APP_MODE_TRACK)
    {
        static int drive_last = -1;
        if (drive_last != app_run)
        {
            Track_SetDrive(app_run ? 1 : 0);
            drive_last = app_run;
        }
    }

#if MISSION_ENABLE
    /* 任务状态机接管: 由它决定这一拍是 循迹 / 盲走 / 定向转 / 停车 */
    if (Mission_IsActive())
    {
        Mission_Run();
        return;
    }
#endif

    if (app_mode == APP_MODE_TRACK) Track_Run();    /* 没开任务状态机 = 纯循迹 */
}

/*---------------------------------------------------------------------------
 * 10ms 任务: 板载按键 S2(PB21) —— 单击 开始/停止, 长按 加速
 *-------------------------------------------------------------------------*/
void APP_Task_10ms(void)
{
    int ev;

    if (key_ev_pending == 0) return;

    /* 取走中断里缓存的事件 (短暂关中断, 免得取+清之间又来了新事件被丢掉) */
    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    ev = key_ev_pending;
    key_ev_pending = 0;
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    if (ev == 0) return;

    /* ---------------- 单击: 开始 / 停止 ---------------- */
    if (ev & KEY_EV_CLICK)
    {
        app_run = app_run ? 0 : 1;

        if (app_run)
        {
            if (app_mode == APP_MODE_TRACK) Track_Reset();  /* 清 PID / 丢线计时 */
            Motor_Enable();                                  /* STBY 使能 */
#if MISSION_ENABLE
            if (app_mode == APP_MODE_TRACK) Mission_Start(); /* 按路线表开跑 */
#endif
#if WORK_FEATURES
            Results_Clear();                                 /* 新任务开始: 清上一次的房间结果 */
#endif
        }
        else
        {
#if MISSION_ENABLE
            Mission_Stop();                                  /* 状态机一起停 (含急停转向) */
#endif
            ImuTurn_Stop();                                  /* 急停: 转向/保向一起停 */
            Motor_Stop();                                    /* 输出清零 + 方向脚复位 */
            PWMA = PWMB = PWMC = PWMD = 0;                    /* 屏上同步归零 */
        }
        /* 这里刻意不发串口: 9600 下发一行日志要 20~40ms, 会把 5ms 循迹节拍卡住 */
    }

    /* ---------------- 长按: 加速一档 ---------------- */
    if (ev & KEY_EV_ACCEL)
    {
        if (app_mode == APP_MODE_TRACK)
        {
            Track_StepSpeed(1);                              /* 150~900, 到顶回绕到 150 */
        }
        else
        {
            target_speed += TARGET_SPEED_STEP;
            if (target_speed > TARGET_SPEED_MAX) target_speed = TARGET_SPEED_MIN;
            Target_A = Target_B = Target_C = Target_D = target_speed;
        }
    }
}

/*---------------------------------------------------------------------------
 * 20ms 任务: 编码器采样 + 闭环速度环
 *-------------------------------------------------------------------------*/
void APP_Task_20ms(void)
{
    IMU_Process();                           /* IMU: 消费环形缓冲, 解析 0x7E 0x23 帧 */
    Encoder_Update();                        /* 换出 4 路 20ms 计数值 */

    A_speed = Motor_Get_Encoder_A();
    B_speed = Motor_Get_Encoder_B();
    C_speed = Motor_Get_Encoder_C();
    D_speed = Motor_Get_Encoder_D();

    if (!app_run)
    {
        return;                              /* 停止状态: 只采样编码器, 不写 PWM */
    }

    if (ImuTurn_IsBusy())
    {
        return;                              /* IMU 转向/保向中: 电机归它, 这里不写 PWM */
    }

    if (app_mode != APP_MODE_MOTOR_TEST)
    {
        return;                              /* 循迹模式: PWM 由 Track_Run() 直接写, 这里不插手 */
    }

    if (stage >= STEP_NUM)                   /* 闭环阶段: 4 路独立速度环 */
    {
        PWMA = Velocity_A(Target_A, A_speed);
        PWMB = Velocity_B(Target_B, B_speed);
        PWMC = Velocity_C(Target_C, C_speed);
        PWMD = Velocity_D(Target_D, D_speed);
    }
    /* 开环阶段 PWMA~PWMD 由 100ms 任务按步骤表写入 */

    Motor_PWM_Front(PWMA, PWMB);
    Motor_PWM_Back(PWMC, PWMD);
}

/*---------------------------------------------------------------------------
 * 100ms 任务: 测试阶段推进 + 写 LCD 文字影子 (真正推屏在主循环 LCD_RowFlush())
 *-------------------------------------------------------------------------*/
void APP_Task_100ms(void)
{
#if IMU_TURN_DEMO
    App_ImuTurnDemo();                       /* 演示: 需把 IMU_TURN_DEMO 改成 1 */
#endif

    if (app_mode == APP_MODE_TRACK)
    {
        LCD_track();
        return;
    }

    if (!app_run)
    {
        LCD_status();                        /* 停止状态: 不推进阶段, 只刷屏 */
        return;
    }

    if (stage < STEP_NUM)
    {
        PWMA = steps[stage].a;
        PWMB = steps[stage].b;
        PWMC = steps[stage].c;
        PWMD = steps[stage].d;

        stage_ms += 100;
        if (stage_ms >= steps[stage].ms)
        {
            stage_ms = 0;
            stage++;                          /* 进入下一步 */
            if (stage >= STEP_NUM)            /* 开环全部走完 -> 清 PID 状态, 进闭环 */
            {
                PWMA = PWMB = PWMC = PWMD = 0;
                Motor_Stop();
                printf("---- open loop done, closed loop ON (target=%d) ----\r\n", target_speed);
            }
        }
    }
    LCD_status();
}

/*---------------------------------------------------------------------------
 * 1000ms 任务: 串口日志
 *-------------------------------------------------------------------------*/
void APP_Task_1000ms(void)
{
    /* 授时模块: 1s 叫一次, 内部按 TIMESYNC_CALIB_MS(已同步=30s) / TIMESYNC_RETRY_MS(未同步=3s)
     * 自己判断这次要不要真读总线 —— 平时空跑不占时间。两次读数之间靠本板晶振走时, 秒不会停。
     * 放这个任务而不是 5ms 控制回路: 读事务是阻塞的 (正常 ~0.6ms, 断线时最坏 50ms 超时)。
     * 从机只在被读时应答、不主动上报, 所以必须自己轮询。 */
    {
        int st;

        TimeSync_Poll();

        st = TimeSync_Status();
        if (st != ts_last_status)      /* 只在状态变化时打串口 (9600 下发一行要几十 ms) */
        {
            ts_last_status = st;

            if (st == 1)
                printf("TIME ok: %s  (utc=%u ok=%u)\r\n", TimeSync_GetStr(),
                       (unsigned int)TimeSync_GetEpochUtc(), (unsigned int)TimeSync_GetOkCount());
            else if (st == 0)
                printf("TIME wait: module not synced yet (bus ok, ok=%u err=%u)\r\n",
                       (unsigned int)TimeSync_GetOkCount(), (unsigned int)TimeSync_GetErrCount());
            else
                printf("TIME FAULT: no ACK on I2C1 PA29/PA30 -> check wire/pullup/power (err=%u)\r\n",
                       (unsigned int)TimeSync_GetErrCount());
        }
    }

//    if (stage >= STEP_NUM)
//        printf("CLOSED  tgt=%d  spd=%d/%d/%d/%d  pwm=%d/%d/%d/%d\r\n",
//               Target_A, A_speed, B_speed, C_speed, D_speed, PWMA, PWMB, PWMC, PWMD);
//    else
//        printf("STEP %2d %-8s pwm=%d/%d/%d/%d  spd=%d/%d/%d/%d\r\n",
//               stage + 1, steps[stage].name, PWMA, PWMB, PWMC, PWMD,
//               A_speed, B_speed, C_speed, D_speed);
}

/*---------------------------------------------------------------------------
 * 系统初始化
 *-------------------------------------------------------------------------*/
static void System_Init(void)
{
    SYSCFG_DL_init();                       /* 时钟/GPIO/PWM/SPI(LCD)/SysTick */

    Motor_Stop();
    Motor_Enable();                         /* TB6612 STBY = 1 使能 */

    Encoder_Init();                         /* 打开 8 个编码器脚的中断 */
    Key_Init();                             /* 板载按键 S1/S2 调速 */
    IMU_Init();                             /* IMU: UART1(PA8/PA9) RX 中断 + RX 上拉 */
    ImuTurn_Init();                         /* IMU 转向/保向层复位 */
    Mission_Init();                         /* 任务状态机复位 (路线表在 middle/Mission.c) */

#if WORK_FEATURES
    TW_Init();                              /* 语音模块控制脚 (PB17/PB18) 置 idle */
    UART_Init();                            /* UART0: 语音模块 (UART_send_char -> 'E'/'F') */
#endif

    Timer_Init();                           /* 1ms 时基: 清标志 + 开中断 */
    DL_TimerG_startCounter(TIMER_0_INST);   /* 启动 1ms 计数 -> 调度器时基 */

    TimeSync_Init();                        /* 授时查询端复位: 等 1000ms 任务去读第一次 */

    LCD_Init();
    /* 上电自检: 刷 3 个纯色 + 清屏, 确认 LCD 在位
     * 用批量版: 整屏 8.2ms/次 (老 LCD_Fill 逐字节要 0.28s/次, 4 次就是 1.1s) */
    LCD_FillFast(0, 0, LCD_W, LCD_H, RED);   delay_ms(200);
    LCD_FillFast(0, 0, LCD_W, LCD_H, GREEN); delay_ms(200);
    LCD_FillFast(0, 0, LCD_W, LCD_H, BLUE);  delay_ms(200);
    LCD_FillFast(0, 0, LCD_W, LCD_H, BLACK);
    LCD_DrawRectangle(0, 0, LCD_W - 1, LCD_H - 1, WHITE);

    if (app_mode == APP_MODE_TRACK)
    {
        Track_Init();                       /* 循迹 PID 初始化 */
#if WORK_FEATURES
        g_bat_v   = Bat_ReadAvg();          /* 上电先读一次电量 (结果区电量行) */
        g_bat_pct = Bat_ToPercent(g_bat_v);
#endif
        printf("\r\n==== TRACK mode: AuroraT8 (I2C0 PA0/PA1), 5ms loop ====\r\n");
        printf("KEY S2/PB21: click = start/stop, long press = speed up\r\n");
        printf("TIME: I2C1 PA30(SDA)/PA29(SCL) -> ESP32-S2 @0x42 400kHz, calib %us\r\n",
               (unsigned int)(TIMESYNC_CALIB_MS / 1000u));
    }
    else
    {
#if SKIP_OPEN_LOOP
        stage = STEP_NUM;                   /* 直接进闭环, 跳过开环自检 */
        printf("\r\n==== Motor test (DEBUG: skip open loop, CLOSED LOOP from boot) ====\r\n");
#else
        printf("\r\n==== Motor test (TB6612, 4 wheels, open loop -> closed loop) ====\r\n");
#endif
        printf("A=LF PA12/PB19/PB20  B=RF PA13/PB4/PB5  C=RB PA17/PB6/PB7  D=LB PA7/PB12/PB13\r\n");
        printf("STBY=PB0  encoder 20ms  target=%d\r\n", target_speed);
    }
}

int main(void)
{
    System_Init();

    while (1)
    {
        Scheduler_Run();
        LCD_RowFlush();     /* 刷 1 行脏行 (<=0.9ms): 100ms 任务只写影子, 推屏在这里分摊 */
    }
}

/*===========================================================================
 * 任务状态机的钩子 (弱函数覆盖 middle/Mission.c 里的空实现)
 *   Mission_OnWork(room): 到达房间/工作区, 已经停车 —— 以后测温/语音/LCD/LORA 挂这里
 *   Mission_OnStepEnter:  每一步开始打一行串口 (台架调路线时很有用)
 *   Mission_OnDone:       全程收工
 * ★ 这些都在 5ms 任务里被调用, 别写长时间阻塞的代码 (要等就用 MS_WORK 步的毫秒数)
 *===========================================================================*/
#if MISSION_ENABLE
void Mission_OnStepEnter(int step)
{
    printf("MSTEP %d %s\r\n", step, Mission_GetStepName());
}

void Mission_OnWork(int room)
{
    /* 到达房间/工作区 (车已停稳): 语音 -> 测温 -> 电量 -> 记结果(屏+时间戳) -> LORA 上报 -> 语音
     * ★ 在 5ms 任务里被调, 但此时车是停着的: 阻塞的 I2C/ADC/串口都在 MS_WORK 的窗口内 */
#if WORK_FEATURES
    uint8_t tok = 0;
    float   temp, bv;
    uint8_t pct;
    char    ts[10], vs[10];
    TimeSync_DateTime dt;

    Voice_0(1);                                     /* 语音: 提示"开始测量" */
    temp = Temp_ReadBody(&tok);                     /* 体温: 多次采样取中位 */
    bv   = Bat_ReadAvg();                           /* 电池电压/电量 (顺手刷新) */
    pct  = Bat_ToPercent(bv);
    g_bat_v   = bv;
    g_bat_pct = pct;

    if ((room >= 1) && (room <= 4))                 /* 记录到结果区 (中途不清) */
    {
        g_room[room].valid   = 1;
        g_room[room].temp_c  = temp;
        g_room[room].bat_v   = bv;
        g_room[room].bat_pct = pct;
        g_room[room].time_ok = (TimeSync_Get(&dt) == 1) ? 1 : 0;
        if (g_room[room].time_ok)
        {
            g_room[room].hh = dt.hour;
            g_room[room].mm = dt.min;
            g_room[room].ss = dt.sec;
        }
    }

    /* LORA 上报 (printf 出口 = LORA 口 UART3/PB2, 9600); 浮点用整数格式化 */
    fmt_temp(ts, temp);
    fmt_volt(vs, bv);
    printf("TEMP,room=%d,temp=%sC,ok=%d,bat=%s,%d%%,time=%s,utc=%u\r\n",
           room, tok ? ts : "--.-", (int)tok, vs, (int)pct,
           TimeSync_GetStr(), (unsigned)TimeSync_GetEpochUtc());

    Voice_1(1);                                     /* 语音: 提示"测量完成" */
#else
    printf("MWORK room=%d t=%s\r\n", room, TimeSync_GetStr());
#endif
}

void Mission_OnDone(void)
{
    /* 收工即停: 跑完最后一步后不要再落回"纯循迹"自己往前跑 (MISSION_DEBUG_STEPS 限流时同理) */
    app_run = 0;
    Motor_Stop();
    PWMA = PWMB = PWMC = PWMD = 0;
#if WORK_FEATURES
    g_task_done = 1;                                /* 屏第 1 行提示"任务完成" (结果不清) */
    {
        char vs[10];
        fmt_volt(vs, g_bat_v);
        printf("DONE,task=complete,rooms=4,bat=%s,%d%%,time=%s,utc=%u\r\n",
               vs, (int)g_bat_pct, TimeSync_GetStr(), (unsigned)TimeSync_GetEpochUtc());
    }
#else
    printf("MDONE done, stopped. t=%s\r\n", TimeSync_GetStr());
#endif
}
#endif
