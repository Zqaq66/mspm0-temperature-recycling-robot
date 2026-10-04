#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_turn_sim.py —— IMU 原地转向控制律的离线模拟器 (纯 stdlib, 不依赖第三方库)

用途: 上车之前先在电脑上跑控制律, 调参数不用反复烧写。
      本文件里的【状态机部分】是 middle/ImuTurn.c 的逐行复刻,
      宏值也照抄 middle/ImuTurn.h —— 改 C 代码时这里要同步改 (两边都标了 ★ 同步)。

车辆模型 (按 2026-10 实测反推, 见 ImuTurn.h 顶部注释):
    驱动:      轮速一阶惯性, 时间常数 TAU_MS, 稳态转速 W_MIN*PWM/120
                (开环 PWM, 只有 120 这一个点标定过: 120 PWM -> 113 度/秒)
    切断(断电+短路制动): 偏航角速度恒定减速, 减速度就是 A_PLANT (度/秒^2)
                => 滑行角 = w^2 / (2*a)   这就是"为什么必须提前切"的根源
    IMU:       25Hz 采样, 角度量化到 0.1 度

用法:
    python imu_turn_sim.py                        # 默认: 同一台车连转 6 次, 看自标定收敛
    python imu_turn_sim.py --deg 90 --trace        # 单次 + 打印事件时间线
    python imu_turn_sim.py --plant-a 300           # 换一台"刹得住"的车
    python imu_turn_sim.py --plant-a 85 --seed-a 145   # 地面变滑, 种子没跟上 (最危险方向)
    python imu_turn_sim.py --sweep                 # 参数扫描表
    python imu_turn_sim.py --trust 0.55            # 试其它 DECEL_TRUST
"""
import argparse
import math

# ===========================================================================
# 宏: ★ 与 middle/ImuTurn.h 保持同步
# ===========================================================================
SPEED_MAX = 170          # IMU_TURN_SPEED_MAX    远段定速 PWM
MIN_PWM = 120            # IMU_TURN_MIN_PWM      近段/脉冲 PWM (静摩擦下限)
SLOW_MARGIN = 8.0        # IMU_TURN_SLOW_MARGIN  "该减速"余量 (度)
CUT_MARGIN = 4.0         # IMU_TURN_CUT_MARGIN   "该切断"余量 (度)
STARTUP_MS = 250         # IMU_TURN_STARTUP_MS   起步保护
RATE_FAST = 250.0        # IMU_TURN_RATE_FAST    满速角速度估计 (度/秒)
DECEL_MIN = 50.0         # IMU_TURN_DECEL_MIN
DECEL_MAX = 4000.0       # IMU_TURN_DECEL_MAX
DECEL_TRUST = 0.70       # IMU_TURN_DECEL_TRUST  ★ 预测用 a 打折 (宁可早切)
DECEL_UP = 0.25          # IMU_TURN_DECEL_LPF_UP     实测 a 更大 -> 慢慢上修
DECEL_DOWN = 0.70        # IMU_TURN_DECEL_LPF_DOWN   实测 a 更小 -> 立刻采纳
PULSE_MAX = 6            # IMU_TURN_PULSE_MAX
PULSE_MIN_MS = 25        # IMU_TURN_PULSE_MIN_MS
PULSE_MAX_MS = 60        # IMU_TURN_PULSE_MAX_MS
PULSE_FRAC = 0.50        # IMU_TURN_PULSE_FRAC   ★ 每拍只补剩余误差的这个比例
GAIN_DEF = 0.25          # IMU_TURN_PULSE_GAIN_DEF  (度/毫秒, 故意高估)
GAIN_MIN = 0.10          # IMU_TURN_PULSE_GAIN_MIN
GAIN_MAX = 1.00          # IMU_TURN_PULSE_GAIN_MAX
GAIN_UP = 0.60           # IMU_TURN_PULSE_GAIN_UP
GAIN_DOWN = 0.20         # IMU_TURN_PULSE_GAIN_DOWN
ARRIVE = 4.0             # IMU_TURN_ARRIVE_DEG   到位窗口 (度)
RATE_MAX = 400.0         # IMU_TURN_RATE_MAX
RATE_STOP = 25.0         # IMU_TURN_RATE_STOP    "算停稳"的角速度阈值 (度/秒)
RATE_STALE = 150         # IMU_TURN_RATE_STALE_MS
ARRIVE_TICKS = 8         # IMU_TURN_ARRIVE_TICKS 停稳确认拍数 (5ms/拍)
RATE_LPF = 0.35          # IMU_TURN_RATE_LPF
ERRWAY_TICKS = 40        # IMU_TURN_ERRWAY_TICKS 越走越远保护拍数
TIMEOUT_MS = 8000        # IMU_TURN_TIMEOUT_MS

TICK_MS = 5.0            # ImuTurn_Update() 由 5ms 任务调用

# 车辆模型默认值 (真实值, 模拟器知道, 控制器不知道)
A_PLANT_DEF = 145.0      # 真实减速能力 (度/秒^2)
TAU_MS = 80.0            # 轮速一阶惯性时间常数
W_MIN = 113.0            # 120 PWM 的稳态角速度 (度/秒)
IMU_DT_MS = 40.0         # IMU 上报周期 (25Hz)


def norm180(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def norm180_turn(d, sgn):
    """±180 是 norm180 的边界 (它返回 -180): 边界值的符号统一到命令方向。
    对应 C 里的 _norm180_turn(); 不这么干, 180 度转向一上来就会误判"冲过目标"。"""
    d = norm180(d)
    if d == -180.0 and sgn > 0:
        d = 180.0
    if d == 180.0 and sgn < 0:
        d = -180.0
    return d


class Car:
    """车辆 + IMU 的物理模型"""

    def __init__(self, a_plant=A_PLANT_DEF, tau_ms=TAU_MS, w_min=W_MIN, yaw0=0.0):
        self.a = a_plant
        self.tau = tau_ms
        self.w_min = w_min
        self.yaw = yaw0         # 真实偏航角 (度, 不量化) —— ★ 必须能非零起步:
                                #   台架上 TURN2 就是从 TURN1 的落点起转的, 而且 ±180 度
                                #   从非零角度起会踩 norm180 的边界 (见 sim() 里 init_dir 的注释)
        self.w = 0.0            # 真实偏航角速度 (度/秒)
        self.t = 0.0            # 秒
        self.pwm = 0            # 当前 PWM (-999..999)
        self.brake = False      # True = 切断驱动 + 短路制动
        self.imu_yaw = round(yaw0, 1)   # IMU 报出来的 yaw (25Hz, 0.1 度量化)
        self._next_imu = IMU_DT_MS / 1000.0
        self.imu_updated = False

    def step(self):
        dt = TICK_MS / 1000.0
        if self.brake:                                    # 切断: 恒定减速度
            if self.w > 0.0:
                self.w = max(0.0, self.w - self.a * dt)
            else:
                self.w = min(0.0, self.w + self.a * dt)
        else:                                             # 驱动: 一阶惯性趋近稳态
            w_ss = self.w_min * self.pwm / 120.0
            self.w += (w_ss - self.w) * (TICK_MS / self.tau)
        self.yaw += self.w * dt
        self.t += dt
        self.imu_updated = False
        if self.t + 1e-9 >= self._next_imu:
            self.imu_yaw = round(self.yaw, 1)
            self._next_imu += IMU_DT_MS / 1000.0
            self.imu_updated = True


def sim(delta_deg, seed_a=A_PLANT_DEF, a_plant=A_PLANT_DEF, trust=DECEL_TRUST,
        tau_ms=TAU_MS, w_min=W_MIN, trace=False, t_max=16.0, yaw0=0.0):
    """跑一次原地转 (从 yaw0 起转 delta_deg)。
    返回 (带符号误差, 事件列表, 末态字典); 误差 > 0 = 冲过目标, < 0 = 差一点。

    ★ init_dir 必须取【实际要往哪边转】(diff 的符号), 不能取 delta 的正负:
      命令 +180 且从非零角度起步时, target=norm180(cur+180) 会让 diff 正好落在
      norm180 的边界 -180 —— 几何上"往负转 180 度"是对的, 但若拿 delta 的正负当基准,
      "冲过目标"判据会立刻为真 -> 250ms 后全速切断 -> 滑 180 度。
      台架上的 TURN2/±180 都是非零角度起步, 所以这条一定要对。
      对应 ImuTurn.c: TurnDeg() 里 init_dir 由初始 diff 的符号决定。"""
    car = Car(a_plant, tau_ms, w_min, yaw0)
    target = norm180(yaw0 + float(delta_deg))
    turn_sign = 1 if delta_deg >= 0 else -1
    init_dir = 1 if norm180_turn(target - norm180(car.imu_yaw), turn_sign) >= 0.0 else -1

    # ---- 控制器状态 (对应 ImuTurn.c 的 static 变量) ----
    decel = seed_a
    gain = GAIN_DEF
    phase = 'FAR'                 # FAR / NEAR / COAST / PULSE
    last_yaw_sample = norm180(yaw0)   # ★ = |Car.imu_yaw| (对应 C 里 last_yaw_sample = cur_yaw),
                                      #   给 0 的话第一帧 dv 会是个大野值
    v_app = 0.0                   # 低通后的带符号角速度
    last_frame_ms = 0.0
    arrive_cnt = errway_cnt = 0
    crossed = False
    cut_rate = cut_yaw = 0.0
    cal_pending = False
    pulse_cnt = 0
    pulse_start = pulse_ms = 0.0
    pulse_dir = 1                 # ★ 脉冲方向在开始时锁死 (对应 C 的 pulse_dir), 中途不翻
    pulse_yaw0 = 0.0
    pulse_cal = False
    last_err = 0.0
    ev = []
    reason = "TIMEOUT"

    while car.t * 1000.0 < t_max * 1000.0:
        car.step()
        now = car.t * 1000.0
        cur = norm180(car.imu_yaw)
        diff = norm180_turn(target - cur, turn_sign)
        ad = abs(diff)
        direction = 1 if diff >= 0.0 else -1

        if now > TIMEOUT_MS:
            reason = "超时"
            break

        # ---- 角速度: 只在"新数据帧"到来时更新 (与 C 一致) ----
        # ★ 判据两边都必须是【归一化后】的 yaw: C 里是 cur_yaw != last_yaw_sample (都过了 _norm180)。
        #   拿原始 yaw 去比归一化值, 一旦原始值越过 +-180 (181 vs -179) 就永远不等 ->
        #   每 5ms 都当成"新帧", dt 被夹到 5ms -> 速率在 900 度/秒的尖峰上乱跳 -> 标定被污染。
        if cur != last_yaw_sample:
            dt = max(5.0, min(200.0, now - last_frame_ms))
            # 当前帧之前 last_frame_ms = 0 的第一次, dt 会很大 -> 被 min(200) 夹住
            v = norm180(cur - last_yaw_sample) * 1000.0 / dt
            last_yaw_sample = cur
            last_frame_ms = now
            v_app += (v - v_app) * RATE_LPF

        # ---- 过期归零 / 夹野值 / 盲走补偿 ----
        v_signed = v_app
        gap = now - last_frame_ms
        if gap > RATE_STALE:
            v_signed = 0.0
        rate = abs(v_signed)
        blind = rate * gap / 1000.0 if gap < RATE_STALE else 0.0
        if rate > RATE_MAX:
            rate = RATE_MAX

        if (diff > 0.0 and init_dir < 0) or (diff < 0.0 and init_dir > 0):
            crossed = True

        a_eff = decel * trust
        coast_pred = rate * rate / (2.0 * a_eff) + blind + CUT_MARGIN

        # ================= 已切断: 等停稳 -> 判落点 -> 脉冲补 =================
        if phase in ('COAST', 'PULSE'):
            if phase == 'PULSE':
                if (now - pulse_start) < pulse_ms and not crossed and ad > 1.5:
                    car.pwm = MIN_PWM * pulse_dir     # SetPWM 会覆盖短路制动; 方向用锁存值
                    car.brake = False
                else:
                    car.pwm = 0
                    car.brake = True
                    phase = 'COAST'
                    pulse_cal = True
                    arrive_cnt = 0
                    if trace:
                        ev.append((round(car.t, 2), "脉冲%d结束" % pulse_cnt,
                                   "t=%dms" % pulse_ms, "err=%.1f" % ad))
                continue
            # COAST
            if rate < RATE_STOP:
                arrive_cnt += 1
                if arrive_cnt >= ARRIVE_TICKS:
                    err = ad
                    if pulse_cal:                     # 标定脉冲增益
                        moved = abs(norm180(cur - pulse_yaw0))
                        if pulse_ms > 0 and moved > 0.2:
                            g = min(GAIN_MAX, max(GAIN_MIN, moved / pulse_ms))
                            gain += (g - gain) * (GAIN_UP if g > gain else GAIN_DOWN)
                        pulse_cal = False
                    if cal_pending:                   # 标定 a
                        coast = abs(norm180(cur - cut_yaw))
                        if cut_rate > 40.0 and 0.5 < coast < 170.0:
                            a = min(DECEL_MAX, max(DECEL_MIN, cut_rate * cut_rate / (2.0 * coast)))
                            decel += (a - decel) * (DECEL_DOWN if a < decel else DECEL_UP)
                        cal_pending = False
                    if trace:
                        ev.append((round(car.t, 2), "停稳", "err=%.1f" % err,
                                   "a=%.0f" % decel, "增益=%.3f" % gain))
                    last_err = err
                    if err <= ARRIVE or crossed or pulse_cnt >= PULSE_MAX:
                        reason = ("到位" if err <= ARRIVE else
                                  ("过冲(不回转)" if crossed else "脉冲用尽"))
                        return norm180(car.yaw - target), ev, dict(
                            err=err, sin_err=norm180(car.yaw - target), a=decel, gain=gain,
                            pulses=pulse_cnt, t=car.t, reason=reason, yaw0=yaw0, target=target,
                            landed=norm180(car.imu_yaw), init_dir=init_dir)
                    t = min(PULSE_MAX_MS, max(PULSE_MIN_MS, (PULSE_FRAC * err) / gain))
                    pulse_ms = t
                    pulse_cnt += 1
                    pulse_dir = direction   # 锁存本拍方向
                    pulse_start = now
                    pulse_yaw0 = cur
                    phase = 'PULSE'
                    arrive_cnt = 0
                    car.pwm = MIN_PWM * direction
                    car.brake = False
                    if trace:
                        ev.append((round(car.t, 2), "打脉冲%d" % pulse_cnt,
                                   "t=%dms" % pulse_ms, "err=%.1f" % err))
                    continue
            else:
                arrive_cnt = 0
            continue

        # ================= 还在驱动 (FAR / NEAR) =================
        if ((v_signed > RATE_STOP and diff < 0.0) or
                (v_signed < -RATE_STOP and diff > 0.0)):     # 转反了 → 远离目标
            errway_cnt += 1
            if errway_cnt >= ERRWAY_TICKS:
                reason = "越走越远(方向反了)"
                break
        else:
            errway_cnt = 0

        # 该切断? (冲过目标 或 剩余角 <= 预测滑行角) —— 只在 NEAR 低速段允许
        if ((crossed or (phase == 'NEAR' and ad <= coast_pred))
                and now >= STARTUP_MS):
            cut_rate = rate
            cut_yaw = cur
            cal_pending = True
            phase = 'COAST'
            car.pwm = 0
            car.brake = True
            arrive_cnt = 0
            if trace:
                ev.append((round(car.t, 2), "切断", "ad=%.1f" % ad,
                           "w=%.0f" % rate, "预测滑行=%.1f" % coast_pred))
            continue

        # 该减速? (单向闩锁; FAR 段用满速估计)
        rate_far = RATE_FAST if phase == 'FAR' else rate
        if phase == 'FAR' and ad <= (rate_far * rate_far / (2.0 * a_eff) + blind + SLOW_MARGIN):
            phase = 'NEAR'
            if trace:
                ev.append((round(car.t, 2), "转低速段", "ad=%.1f" % ad))

        car.pwm = (MIN_PWM if phase == 'NEAR' else SPEED_MAX) * direction
        car.brake = False
        arrive_cnt = 0

    return norm180(car.yaw - target), ev, dict(
        err=abs(norm180(target - norm180(car.imu_yaw))), sin_err=norm180(car.yaw - target),
        a=decel, gain=gain, pulses=pulse_cnt, t=car.t, reason=reason,
        yaw0=yaw0, target=target, landed=norm180(car.imu_yaw), init_dir=init_dir)


def one(args):
    e, ev, st = sim(args.deg, seed_a=args.seed_a, a_plant=args.plant_a, trust=args.trust,
                    tau_ms=args.tau, w_min=args.w_min, trace=args.trace, yaw0=args.yaw0)
    print("=" * 74)
    print("起点 %+.1f°   命令 %+d°   目标 %+.1f°   真车 a=%.0f°/s²   种子 a=%.0f   TRUST=%.2f"
          % (args.yaw0, args.deg, st['target'], args.plant_a, args.seed_a, args.trust))
    et = e * st['init_dir']    # 按【行进方向】看的误差: >0 = 越过目标, <0 = 差一点
    print("落点 %+.1f°   误差 %+.2f° (行进方向: %s)   脉冲 %d 拍   用时 %.2fs   学到 a=%.0f   [%s]"
          % (st['landed'], e, "越过目标" if et > 0 else "差一点",
             st['pulses'], st['t'], st['a'], st['reason']))
    if ev:
        print("-" * 74)
        print("t(s)   事件          细节")
        for it in ev:
            print("  %-5s %-13s %s" % (it[0], it[1], "  ".join(str(x) for x in it[2:])))


def sequence(args):
    """同一台车连转多次: 看自标定能不能收敛 (种子 = 上一次学到的 a)"""
    seq = [90, -90, 180, -180, 90, 180]
    print("=" * 74)
    print("同一台车连转 %d 次 (起点=上一转落点)   真车 a=%.0f   DECEL_TRUST=%.2f"
          % (len(seq), args.plant_a, args.trust))
    print("=" * 74)
    seed = args.seed_a
    yaw = args.yaw0        # ★ 连转时起点 = 上一转的落点 (台架就是这样)
    errs = []
    for k, d in enumerate(seq, 1):
        a_before = seed                     # 这一转开始前学到的 a
        e, ev, st = sim(d, seed_a=seed, a_plant=args.plant_a, trust=args.trust,
                        tau_ms=args.tau, w_min=args.w_min, yaw0=yaw)
        seed, yaw = st['a'], st['landed']
        errs.append(abs(e))
        print("  第%d转 %+5d° 起 %+7.1f° -> 目标 %+7.1f° 落点 %+7.1f°  误差 %5.2f°  "
              "脉冲 %d 拍  %5.2fs  a: %.0f -> %.0f   [%s]"
              % (k, d, st['yaw0'], st['target'], st['landed'], abs(e),
                 st['pulses'], st['t'], a_before, st['a'], st['reason']))
    print("-" * 74)
    print("  最差 %.2f°   平均 %.2f°" % (max(errs), sum(errs) / len(errs)))


def sweep(args):
    """参数扫描: 真车 a 在 85~300 之间变化 (地面/电量), 看最差误差"""
    print("=" * 74)
    print("地面/电量变化扫描 (每个格子 = 连转 90/180/-90 的最差误差, 种子从上一转继承)")
    print("=" * 74)
    for trust in (0.85, 0.70, 0.60, 0.50):
        row = []
        for a_plant in (85, 145, 300):
            seed = args.seed_a
            worst = 0.0
            for d in (90, 180, -90):
                e, ev, st = sim(d, seed_a=seed, a_plant=float(a_plant), trust=trust,
                                tau_ms=args.tau, w_min=args.w_min)
                seed = st['a']
                worst = max(worst, abs(e))
            row.append("真车a=%3d: %5.2f°" % (a_plant, worst))
        print("  TRUST=%.2f   %s" % (trust, "   ".join(row)))


def main():
    ap = argparse.ArgumentParser(description="IMU 原地转向控制律离线模拟器 (镜像 ImuTurn.c)")
    ap.add_argument("--deg", type=int, default=90, help="转多少度 (可负)")
    ap.add_argument("--seed-a", type=float, default=A_PLANT_DEF, help="控制器初始 a (IMU_TURN_DECEL_DEF)")
    ap.add_argument("--plant-a", type=float, default=A_PLANT_DEF, help="真车 a (度/秒^2)")
    ap.add_argument("--trust", type=float, default=DECEL_TRUST, help="IMU_TURN_DECEL_TRUST")
    ap.add_argument("--tau", type=float, default=TAU_MS, help="轮速时间常数 (ms)")
    ap.add_argument("--w-min", type=float, default=W_MIN, help="120PWM 的稳态角速度 (度/秒)")
    ap.add_argument("--yaw0", type=float, default=0.0, help="起始 yaw (度), 台架上 TURN2 是非零起步")
    ap.add_argument("--trace", action="store_true", help="打印事件时间线")
    ap.add_argument("--seq", action="store_true", help="连转 6 次 (默认模式)")
    ap.add_argument("--sweep", action="store_true", help="扫描地面/电量变化")
    args = ap.parse_args()

    if args.sweep:
        sweep(args)
    elif args.seq:
        sequence(args)
    else:
        one(args)


if __name__ == "__main__":
    main()
