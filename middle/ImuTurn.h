#ifndef __IMU_TURN_H__
#define __IMU_TURN_H__

/*===========================================================================
 * IMU 转向 / 保向层 -- 用 YbIMU 的偏航角(yaw) 做两件事:
 *   1) 转到指定角度 (原地差速旋转, 到位自动停车)
 *   2) 保持指定角度 (边走边差速纠偏; speed=0 就是原地锁角)
 *
 * 仿 Competition_PJ/BSP/IMU/imu_turn.c, 按本工程电机接口重写
 *   (Motor_SetLeftPWM/SetRightPWM +-999, 差分驱动); 参考里的"转到黑线就停"没有搬。
 *
 * ---------------------------------------------------------------------------
 * 与其它模块的仲裁 (重要, 不会互相抢电机)
 *   用 ImuTurn_IsBusy() 做唯一判据, 工程里已经接好:
 *     - 5ms 任务: 忙 -> 调 ImuTurn_Update() 且不跑 Track_Run(); 结束那一拍自动
 *       Track_Reset(), 循迹下一拍接管 (不会带着旧 PID/丢线状态)
 *     - 20ms 任务: 忙 -> 直接 return, 不写 PWM (电机测试的开环/闭环也让出)
 *     - 单击 S2 停止 (app_run=0) -> ImuTurn_Stop(), 电机立刻停
 *     - 转向/保向期间不读灰度/I2C, 不和循迹抢 I2C 总线
 *   另外: 启动前会检查 IMU 有没有在收数据, 没有就返回 -1 不启动;
 *         运行中超过 IMU_TURN_IMU_OFF_MS 收不到字节 -> 自动停车 (报告 IMU_OFF)
 *
 * ---------------------------------------------------------------------------
 * 用法
 *   ImuTurn_Init();                      // System_Init() 里已经调过
 *
 *   // (1) 原地左转 90 度 (0 = 用 IMU_TURN_SPEED_MAX), 5ms 任务自动推进, 到位自动停
 *   if (ImuTurn_TurnDeg(+90, 0) == 0) { ...启动成功... }
 *   //     轮询完成: if (ImuTurn_JustFinished()) { 原因见 ImuTurn_GetDoneReason(); }
 *
 *   // (2) 保持航向 90 度, 边走边修 (250 是前进 PWM)
 *   ImuTurn_HoldAbs(90.0f, 250);
 *   ImuTurn_HoldAbs(90.0f, 0);           // 原地锁角: 被推动会自己转回来
 *
 *   // (3) 放弃/急停
 *   ImuTurn_Stop();
 *
 *   完成回调 (可选): ImuTurn_SetDoneCallback(my_cb);   // void my_cb(int reason)
 *
 * ---------------------------------------------------------------------------
 * 调参入口 (都在这里, 改完重新编译)
 *===========================================================================*/

/* ---- 完成原因 ---- */
#define IMU_TURN_DONE_ARRIVED   0   /* 转到目标角度          */
#define IMU_TURN_DONE_STOPPED   1   /* 被 ImuTurn_Stop() 停  */
#define IMU_TURN_DONE_IMU_OFF   2   /* IMU 收不到数据, 异常停 */
#define IMU_TURN_DONE_TIMEOUT   3   /* 超时, 异常停          */
#define IMU_TURN_DONE_ERRWAY    4   /* 越走越远(方向反了/被顶住), 已停车 */

/* ---- 当前模式 ---- */
#define IMU_TURN_IDLE           0
#define IMU_TURN_SPOT           1   /* 原地转到指定角度 */
#define IMU_TURN_HOLD           2   /* 保持指定航向     */

/* ---- 坐标系 ---- */
/* ---- 方向符号 (重要, 别乱动) ----
 * 电机侧约定: z > 0 = 左轮正转/右轮反转 = 左转(逆时针)。
 * 实测: 命令 +90 时, IMU 报的 yaw 一路往【负】方向跑 (0 -> -130), 说明
 *   **IMU 的 yaw 是"顺时针为正"(罗盘式, 与右手系相反)**, 和驱动方向反着。
 * 所以这里必须是 -1: cur_yaw = -euler[2], 这样 z>0 才会让 cur_yaw 变大(朝目标走)。
 * 屏上显示的 yaw 已按这个符号校正过 -> 屏上 "+" 就是"命令里的正方向"。
 * 【教训】转向方向不对先查这个符号, 不要先去改控制律。 */
#define IMU_TURN_YAW_SIGN      (-1)

/* ---- 原地转向 (TurnDeg) 控制思路 (v6: 纯惯性滑行停) ----
 *   远段定速转 -> 近段(剩余 < SLOW_FROM_DEG)降到 MIN_PWM 低速 -> 到提前角切断驱动
 *   -> 靠惯性滑到目标 -> 停稳判定落点 -> (停早了) 用最小 PWM 单向补足。
 *   不做任何"回头修正", 也不做反向出力制动 —— 就是"提前停转 + 惯性到位"。
 *
 * 【为什么必须先把速度降下来】
 *   麦轮原地转**轮子刹不住转向**(接触点速度沿辊子自由方向, 锁死轮子辊子照样滚),
 *   切断驱动后只能靠地面/辊子阻尼慢慢磨。滑行角 = w^2 / (2a):
 *     - 实测从全速(170 PWM, w 约 250 度/秒)切断 -> 滑了 47 度
 *     - 按此反推 a 约 665 度/秒^2
 *     - 从 MIN_PWM(120, w 约 100 度/秒)切断 -> 滑约 7.5 度   <-- 这就是近段降速的意义
 *   => 提前角给 8~12 度就够; 给 40 反而会跳过低速段、从全速直接切进去, 必然滑几十度。
 *
 * 【纯惯性 vs 主动制动】
 *   IMU_TURN_BRAKE_PWM = 0 -> 纯惯性滑行 (当前, 你要的)。落点精度受地面/电量影响, 估计 +-5~10 度。
 *   IMU_TURN_BRAKE_PWM > 0 -> 到提前角后电机反向出力把转速拉下来 (更稳、与地面无关),
 *                             但会在到达目标前给一段反向力矩。要恢复就把它设成 200~300。
 *
 * 【任意起点 / 任意方向】
 *   target = norm180(cur_yaw + delta), diff = norm180(target - cur_yaw)
 *   -> 从任何角度起、delta 正负都可以; delta = +-90 / +-180 都支持。
 *   cur_yaw 已归一化到 (-180, 180], 屏上/GetYaw() 读出来不会有 -350 这种值。
 *   ★ 方向由 IMU_TURN_YAW_SIGN 决定, 见下面的说明。 */

#define IMU_TURN_SPEED_MAX      170    /* 远段定速 PWM */
#define IMU_TURN_SLOW_FROM_DEG  35.0f  /* 剩余角 < 此值 -> 降到 IMU_TURN_MIN_PWM 低速接近。
                                        * ★ 这段是"纯惯性停"的关键: 把进入滑行段的速度先降下来,
                                        *   滑行距离正比于转速的平方, 速度减半滑行距离只剩 1/4 */
#define IMU_TURN_MIN_PWM        120    /* 低速段/补足段 PWM (也是静摩擦下限, 卡着不动就加大) */
#define IMU_TURN_LEAD_DEG       35.0f  /* **提前刹车角**: 离目标还剩这么多度就切断驱动, 靠惯性滑到目标。
                                        * 冲过头 -> 加大; 停不到目标 -> 减小。
                                        * 90 度和 180 度通用 (滑行距离只取决于切断时的转速) */
#define IMU_TURN_BRAKE_PWM        0    /* **反向出力制动: 0 = 关闭 (按你的要求, 默认纯惯性滑行)**。
                                        * 设 >0 (= 接近角速度 100度/秒 时给的 PWM) 才启用主动制动,
                                        * 详见下面"纯惯性 vs 主动制动"的说明 */
#define IMU_TURN_BRAKE_MAX      400    /* 主动制动的出力上限 (只有 BRAKE_PWM>0 时才有用) */
#define IMU_TURN_LEAD_MIN_DEG    5.0f  /* 补足段(第一次停早了才进)的提前刹车角。
                                        * 要 <= IMU_TURN_ARRIVE_DEG, 否则补足段会停在窗口外面 */
#define IMU_TURN_RATE_STOP      25.0f  /* 角速度(度/秒)小于它 = 转速已很低, 交回断电保持 */
#define IMU_TURN_RATE_STALE_MS   150   /* 超过这么久没有新的 yaw 帧 = 车没在动, 角速度按 0 算
                                        * (IMU 静止时 yaw 值可能一模一样, 不算这条会卡住不走) */
#define IMU_TURN_ERRWAY_TICKS    40    /* 驱动段却连续这么多拍(30x5ms=150ms)在"远离目标" ->
                                        * 判定方向反了/被顶住, 立即停车 (防白转一大圈) */
#define IMU_TURN_ARRIVE_DEG      5.0f  /* 停稳后落点认到位的窗口 (度) */
#define IMU_TURN_ARRIVE_TICKS   10     /* 停稳后连续确认多少拍(5ms) -> 10 拍 = 50ms */
#define IMU_TURN_RATE_LPF     0.35f    /* 角速度一阶低通 (0~1, 越小越平滑) */
#define IMU_TURN_TIMEOUT_MS     6000   /* 超时自动停 (0 = 不限时) */
#define IMU_TURN_KP             25    /* 保留: 已不被 v4 控制律使用 (定速段是恒速, 没有 P 可调) */

/* ---- 保持航向 (HoldAbs) ---- */
#define IMU_HOLD_KP             8     /* PWM / 度。画龙就减小, 纠不回来就加大 */
#define IMU_HOLD_DEADBAND_DEG   3.0f  /* 死区, 小偏差不修 (防抖) */
#define IMU_HOLD_ZLIM_PCT       80    /* 差速量上限 = 前进速度的 80% */
#define IMU_HOLD_ZMAX           300   /* speed=0 (原地锁角) 时的差速上限 */
#define IMU_HOLD_TIMEOUT_MS     0     /* 0 = 一直保持到 ImuTurn_Stop() */

/* ---- 保护 ---- */
#define IMU_TURN_IMU_OFF_MS     300   /* 超过这么久没收到 IMU 字节 -> 停车 */

/*===========================================================================
 * 接口
 *===========================================================================*/
void    ImuTurn_Init(void);
void    ImuTurn_SetDoneCallback(void (*cb)(int reason));

/* 转到指定角度: delta_deg 正=左转(逆时针), 负=右转; speed = 旋转 PWM (<=0 用 IMU_TURN_SPEED_MAX)
 * 返回 0=启动成功, -1=正忙 / IMU 没数据 */
int     ImuTurn_TurnDeg(int delta_deg, int speed);

/* 保持指定角度: abs_yaw_deg = 绝对航向(度), speed = 前进 PWM (0 = 原地锁角)
 * 返回 0=启动成功, -1=正忙 */
int     ImuTurn_HoldAbs(float abs_yaw_deg, int speed);

void    ImuTurn_Stop(void);            /* 放弃当前动作 (电磁制动) */
void    ImuTurn_Update(void);          /* 由 5ms 任务调用, 推进状态机 */
int     ImuTurn_IsBusy(void);          /* 1 = 正在转向/保向 (此时它独占电机) */
int     ImuTurn_JustFinished(void);    /* 本拍刚结束? 读后自动清零 */
int     ImuTurn_GetDoneReason(void);   /* 上一次结束原因 */

/* 调试/显示 */
float   ImuTurn_GetYaw(void);          /* 当前 yaw (度) */
float   ImuTurn_GetDiff(void);         /* 剩余角度 (度, 正=还要左转) */
float   ImuTurn_GetTarget(void);       /* 目标角度 (度) */
int     ImuTurn_GetMode(void);         /* IMU_TURN_IDLE / SPOT / HOLD */
float   ImuTurn_GetRate(void);         /* 当前接近角速度 (度/秒), 看它是不是 >RATE_STOP 就知道停没停 */
char    ImuTurn_GetPhase(void);        /* 本拍在干什么: D=远段定速 C=低速接近 R=反向制动 B=断电保持 F=补足 */

/* 运行期调参 (不常用, 调好写回头文件宏即可) */
void    ImuTurn_SetGains(int kp_turn, int min_pwm, int kp_hold);

#endif
