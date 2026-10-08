/*===========================================================================
 * 任务状态机 (移动控制层) —— 循迹 -> 检测路口 -> 判方向 -> 转向 -> 停车 -> 恢复循迹
 *
 * 逻辑全在 Mission_Run() (5ms 调一次); "路线"全在下面的 route[] 表里。
 * 改路线 = 改表; 加功能 = 在 MS_WORK 步里挂 Mission_OnWork()。
 *
 * ---------------------------------------------------------------------------
 * ★★★ 下表是【骨架示例】, 转弯方向/判据请按实际场地标定后再定稿 ★★★
 *      地图(项目7 图1)上的位置参考:
 *        左下 = 模拟停车场(出发)   左中 = 丁字口(往右 2m 是充电区支线)
 *        左上角 = 直角弯           顶部横线
 *        右上 = 1号房间口(竖线往下挂 2/3/4 号房间, 各一个口)
 *        右下角 = 直角弯           右侧 3m 竖线拐回顶部
 *      参考实现(第四组)走的是: 出发 -> 直行穿过左中口 -> 左上角右转 ->
 *        沿顶边直行到 1 号房间 -> 停车测温 -> 掉头 -> ... (他们的方向/角度按他们场地调的)
 * ---------------------------------------------------------------------------
 * 每行的列: 动作 | arg | 退出条件 | 条件参数 | 停车ms | 超时ms | 房间号 | 名字
 *   动作: MS_TRACK 循迹 / MS_BLIND 盲走直行 / MS_TURN IMU定角转 / MS_WAIT 停等
 *         MS_WORK 工作环节(停车+调钩子) / MS_END 收工
 *   退出: MS_EX_CROSS 路口 / MS_EX_ROOM 房间黑块 / MS_EX_TIME 计时 / MS_EX_TURN 转完
 *         / MS_EX_LINE 指定路重新压线 / MS_EX_NONE 只能靠超时
 * ============================================================================*/
#include "Mission.h"
#include "XunJi.h"
#include "ImuTurn.h"
#include "Motor.h"
#include "Timer.h"
#include "stdio.h"

#define MS_BLIND_SPEED   250    /* MS_BLIND 没给 PWM 时的直行速度 */
#define MS_BLIND_THRU_MS 600    /* 岔路口"不转"时盲走直行穿过去口的时长 (按车速/口宽标定) */

/* ---- 转弯 (2026-10-06) ----------------------------------------------------
 * MS_ADV_MS: 转弯前先盲走直行这么久, 再原地转。
 *   ★ 为什么必须前移: 8 路灰度装在小车最前方, 传感器"看到路口/弯"时车的中心还没到
 *     路口, 还差一点距离。马上原地转的话, 转完传感器就落到线外 -> 车找不到线。
 *     先直走 MS_ADV_MS 把这段距离补回来, 转完传感器正好还在线的上方。
 *   ★ 标定: 拿尺量"传感器排到两轮中心"的距离 d, 用 d / 实际车速 估一个值再实测微调。
 *     走过头(转完传感器在线前面)就调小, 没走到(转完传感器在线后面)就调大。
 * ------------------------------------------------------------------------- */
#define MS_ADV_MS       250    /* 转弯前盲走直行的时长 (500 偏长 -> 350 -> 250; 余量由转完的"找线"兜) */
#define MS_TURN_DEG      90    /* 转弯角度: 本场地所有转弯都是 90 度 */

/* 传感器判出来的侧向 (+1 右 / -1 左) 怎么变成 ImuTurn 的角度 (全工程只此一处映射):
 *   * 以【实测】为准: 2026-10-06 上板验证 delta 正 = 右转(顺时针), 负 = 左转。
 *     (ImuTurn.h 里的注释写的是"正=左转", 与实测相反 -> 按用户约定: 注释不可信, 改这里。)
 *   * 万一换了车/接了别的 IMU 后方向又反了, 只翻这一个宏, 不要去动 ImuTurn。 */
#define MS_TURN_DIR_FLIP   1    /* 1 = 按实测 (正=右转); 0 = 按 ImuTurn 注释 (正=左转) */
#if MS_TURN_DIR_FLIP
#  define MS_DEG_RIGHT   (+90)   /* 右转 */
#  define MS_DEG_LEFT    (-90)   /* 左转 */
#else
#  define MS_DEG_RIGHT   (-90)   /* 右转 */
#  define MS_DEG_LEFT    (+90)   /* 左转 */
#endif

/* ---- 转弯的"预防"设计 (2026-10-06 用户原则: 找线只是异常处理, 重点是【别产生异常】) ----
 *   [1] 前移期间也读线 (MS_ADV_READ): 主线在前移中就整段消失 = 其实不是岔路而是直角弯 (b)
 *   [2] ★转角出口闭环 (MS_TURN_SENSOR_EXIT): 收工条件 = "细线回到阵列正中"(Track_LineCentered),
 *       IMU 到角只是兜底 —— 打滑/场地不是正 90 度都能自纠, 落点由传感器保证。(c, 预防主力)
 *   [3] 兜底才找线 (g): 直进 -> 首选侧弧线 -> 镜像弧线 -> 直退 -> 原地扫 -> NOLINE
 * 为什么 [2] 有效: 传感器在车前 d 处, 绕车中心转 θ 时线在阵列上的位置
 *   y = -(O + d*cosθ)/sinθ   (O = 落点横向误差) -> y=0 才是"车轴正压线";
 *   转过 ~60 度之前不可能满足(会被旧线/口子黑区骗), 所以加 MS_TURN_SENSOR_DEG 门槛。 */
#define MS_ADV_READ         1     /* 1 = 前移期间也读线 */
#define MS_ADV_GONE_FRAMES  3     /* 前移中连续这么多帧全白 -> 主线没了 = 直角弯 */
#define MS_TURN_SENSOR_EXIT 1     /* 1 = 转角出口优先用"细线到阵列正中" (IMU 兜底) */
#define MS_TURN_SENSOR_DEG 60     /* 转过这么多度之后才允许传感器出口 (小角度用) */
#ifndef MS_TURN_SENSOR_TAIL
#define MS_TURN_SENSOR_TAIL 25    /* ★大角度还要"离目标角不足这么多度"才允许传感器出口:
                                   *   90 度 -> 门槛 60;  180 度 -> 门槛 155。
                                   * 为什么 180 度必须这样: 掉头时线本来就会【合法地】扫过阵列正中
                                   * 好几次 (房间黑区边缘、支线在阵列下掠过), 而 60~90 度正好是掉头的
                                   * 正中间 -> 用 60 度门槛会在半路把掉头掐断。
                                   * (2026-10-07 上板实测: MS_TURN 改走转弯序列后掉头只转一半) */
#endif
#define MS_FIND_SPEED     180     /* 找线时前进/后退的 PWM (慢, 别冲过线) */
#define MS_FIND_MS        600     /* 直退找线时长 */
#define MS_FIND_MS_SHORT  250     /* (1) 直进找线: 便宜试探, 线多半就在脚下/近处 */
#define MS_ARC_SPEED      160     /* (2)(3) 弧线找线: 前进 PWM */
#define MS_ARC_BIAS        60     /* 弧线弯度 = 内外轮差速 (太小横向只走 1cm 级, 太大变原地转) */
#define MS_ARC_MS         700     /* 单侧弧线走多久 (左右各走一次) */
#define MS_ARC_FIRST_SIDE   1     /* 先往哪边画弧: +1 右 / -1 左 (另一侧随后镜像) */
#define MS_SCAN_DEG        60     /* 原地扫描范围: 正负这么多度 */
#define MS_SCAN_STEP       15     /* 每步转多少度 (别超过 20) */
#define MS_SCAN_SETTLE     60     /* 每步转完停稳多久才读线 (ms) */

/* ---- MS_TURN 的两种走法 (2026-10-07 用户定的规则, 别再混) ---------------
 *   (a) 180 度掉头 (|arg| >= 180): 【直接原地转】—— 不前移、不进转弯序列、不找线。
 *       掉头前加那 250ms 前移会把车带出工作位置 -> 掉头就做不成了 (2026-10-07 上板实测)。
 *       几何上也不需要: 绕几何中心转 180 度, 传感器从"车前 L"变到"车后 L",
 *       线是顺着航向走的, 前后挪 2L 不影响它压不压线。
 *   (b) 90 度 (MS_TURN_MEMO, 从房间支线回上一个路口): 【走转弯序列】——
 *       先盲走 MS_TURN_ADV_MS 再原地转 90, 转完再看一眼线 (找线兜底)。
 *       这个 T 字口和 180 掉头一样是"大面积黑", 但 90 度不补这段, 转完传感器就落在线外。
 * ----------------------------------------------------------------------- */
#define MS_ADV_FAIL_MS    500    /* 前移期 IMU 一直起不来(没数据/正忙)的额外宽限 -> 停车报警
                                  * (掉头不前移时也有 500ms 起转宽限, 别太紧) */
#define MS_SEQ_MAX_MS   15000    /* 整段转弯序列(前移+转+找线)的总看门狗: 超时停车报警 */
#ifndef MS_TURN_ADV_MS
#define MS_TURN_ADV_MS  MS_ADV_MS /* MS_TURN 步(180 掉头/回主路)的前移时长; 0 = 不前移 (现场对比用) */
#endif

/*===========================================================================
 * ★ 怎么按场地填这张表 (2026-10-07)
 *
 * 0) 一句话: 表 = 【车按顺序走过的每一段】; 一行 = 一段路 + 一个"什么时候算走完"。
 *
 * 1) 先数场地上的"要停 / 要拐的地方" (地图: 项目7 图1):
 *      出发点(停车场) / 充电区 / 4 个房间口 / 1 个丁字口 / 2 个直角弯
 *
 * 2) 每段路用哪个动作:
 *      沿黑线走一段         -> MS_TRACK   (退出条件写"走到哪")
 *      不看线直走一段       -> MS_BLIND   (arg = PWM, 0 = 用默认 250)
 *      要定角拐弯           -> MS_TURN    (arg = 角度, 见第 4 条)
 *      停车等时间           -> MS_WAIT    (exit_arg = 毫秒)
 *      停车做事(测温/LORA)  -> MS_WORK    (exit_arg = 毫秒, room = 房间号)
 *      结束                 -> MS_END
 *
 * 3) 退出条件怎么选 (决定"这段路什么算走完"):
 *      MS_EX_ROOM   大面积黑 (>=6 路压线) = 房间口 / 工作区 / 充电区
 *      MS_EX_JUNC   判据同 MS_EX_ROOM, 只是名字贴切: 从支线出来撞上主线那个 T 字口
 *      MS_EX_CROSS  最左或最右压线 = 岔路口 (MS_CROSS 步用)
 *      MS_EX_TIME   计时到 exit_arg 毫秒
 *      MS_EX_TURN   转向做完 (MS_TURN 步用)
 *      MS_EX_LINE   指定的那几路重新压线 (exit_arg = 掩码, bit=0 表示压线; 例 0xC0 = bit6/7)
 *      MS_EX_NONE   自己不退, 只能靠 timeout_ms 兜
 *
 * 4) 三种"拐弯"分别怎么写 (★ 最容易填错的地方):
 *      普通直角弯          : 【不写表!】车在 MS_TRACK 步里自己判出来, 自己"前移 250ms + 转 90"。
 *      岔路口              : 写一行 MS_CROSS ——
 *                              arg = 1 : 要拐进支线 (往左还是往右由传感器判, 表里不写方向)
 *                              arg = 0 : 直行穿过这个口 (盲走 MS_BLIND_THRU_MS 毫秒)
 *                            ★ 每个岔路口单独占一行, 走完这一步才继续下一步。
 *      180 度掉头          : 写一行 { MS_TURN, +180, MS_EX_TURN, ... } -> 【原地直接转, 不前移】。
 *      从支线回主路的 T 字口: 写一行 { MS_TURN, MS_TURN_MEMO, MS_EX_TURN, ... } ->
 *                            盲走 MS_TURN_ADV_MS(250ms) + 原地转 90, 方向 = 进支线时那个方向。
 *                            ★ 这两个口在传感器眼里都是"大面积黑 T 字口" (判据一样),
 *                              区别只在转的角度: 90 度要前移, 180 度不前移 (见 turn_seq_start)。
 *
 * 5) 一个房间 = 连着的 5 行; 2/3/4 号房间照抄房间 1 这组, 只改 room 号和名字:
 *      行1 MS_TRACK  MS_EX_ROOM   循迹到房间口 (大面积黑)
 *      行2 MS_WORK   MS_EX_TIME   停车做事: room = N, exit_arg = 1500ms (测温/LCD/LORA 挂这里)
 *      行3 MS_TURN   +180         原地掉头 (测完原样转回来)
 *      行4 MS_TRACK  MS_EX_JUNC   沿支线循迹回上一个路口 (撞上主线 = 大面积黑)
 *      行5 MS_TURN   MS_TURN_MEMO 转回主路方向 (方向和进支线时同侧)
 *      ★ 房间口和"回主路撞到的 T 字口"在传感器眼里一模一样 (都是大面积黑), 全靠它们在表里的
 *        【位置】区分 -> 这 5 行的顺序不能乱 (行1 是房间口, 行4 才是路口的 T 字口)。
 *      ★ 几个房间共用同一个"上一个路口"时, 每个房间前面那一行 MS_CROSS 都要按地图重写一遍。
 *
 * 6) 每行的两个时间 (最容易混):
 *      stop_ms    = 这一步【做完之后】在原地停多久再进下一步 (属于本行, 不是下一行)
 *      timeout_ms = 兜底: 退出条件一直不成立也往下走, 防卡死
 *                   (180 掉头行用它; 90 度那种走转弯序列的行由 MS_SEQ_MAX_MS 看门狗兜)
 *      收工前那三行 (回充电区 / 停 5s / 回停车场) 照抄表里现成的 CHG / CHG5S / PARK。
 *
 * 7) 填完怎么验 (台架, 别一次跑全表):
 *      把下面的 MISSION_DEBUG_STEPS 改成 1 -> 只跑第 0 行, 验"循迹到第一个岔路口 + 自判方向拐过去";
 *      一段验通了把 N 加 1, 一段一段往下推 (也可以 Keil Misc Controls 加 -DMISSION_DEBUG_STEPS=2)。
 *      每步进出串口都会打一行 (MSTEP n 名字); 屏第 1 行 M<步号><步名><子状态> 看现场卡在第几步。
 *===========================================================================*/
static const MissionStep route[] =
{
/* 动作             arg       退出条件      条件参数   停车ms 超时ms 房间  名字
 * ---------------------------------------------------------------------------
 * ★ 转弯方向一律不写死: MS_CROSS 的 arg 只说"要不要转"(1转/0直行穿过), 往哪边转
 *   由传感器自己判 (XunJi 的岔路判定 -> track_branch_side)。
 *   MS_TURN 只用在必须指定方向的地方: 180 掉头 / 从房间支线回主路 (MS_TURN_MEMO)。
 *   普通弯不在表里: 主线自己拐走的弯, 车在 MS_TRACK 步里自动"前移 MS_ADV_MS + 转 90"。
 *   ★ MS_TURN 步也走同一套转弯序列 (前移+转+找线) -> 那一行的 timeout_ms 不再起作用,
 *     整段由 MS_SEQ_MAX_MS 看门狗兜 (见文件上面 MS_ADV_FAIL_MS/MS_SEQ_MAX_MS)。
 * ------------------------------------------------------------------------- */
/* ---- 下面这张表: 按用户 2026-10-07 口述的路线填 (左右以【小车视角】为准) ---------
 *   每行 = 一步; 行尾注释 = 用户说的那句话。
 *   普通拐弯 / 直行段不占行 (普通弯由 MS_TRACK 自己转, 直行是下一行的循迹段)。
 * ---------------------------------------------------------------------------- */
/* -- 出发: 直行, 直行经过第 1 个岔路口 (不拐, 穿过) --------------------- */
  { MS_CROSS,        0,   MS_EX_NONE,       0,      400,  12000,  0,  "J1"   },
/* -- 直行 (中间一个普通弯右拐) -> 第 2 个岔路口: 直行穿过 -> 去 1 号房间 - */
  { MS_CROSS,        0,   MS_EX_NONE,       0,      400,  12000,  0,  "J2"   },
/* -- 循迹到 1 号房间口 ---------------------------------------------------- */
  { MS_TRACK,        0,   MS_EX_ROOM,       0,      400,  12000,  0,  "T1"   },
/* -- 1 号房间: 停车做事 (测温/语音/LCD/LORA 都挂这个钩子) ----------------- */
  { MS_WORK,         0,   MS_EX_TIME,    1500,      200,   8000,  1,  "WORK1"},
/* -- 掉头 -> 沿支线回上一个路口 -> 转回主路 ------------------------------ */
  { MS_TURN,      +180,   MS_EX_TURN,       0,      200,   9000,  0,  "U180" },
  { MS_TRACK,        0,   MS_EX_JUNC,       0,      400,   9000,  0,  "BACK1"},
  { MS_TURN,  MS_TURN_MEMO, MS_EX_TURN,     0,      200,   6000,  0,  "M90_1"},
/* -- 直行 -> 岔路口拐弯进 2 号房间 --------------------------------------- */
  { MS_CROSS,        1,   MS_EX_NONE,       0,      400,  12000,  0,  "J3"   },
  { MS_TRACK,        0,   MS_EX_ROOM,       0,      400,  12000,  0,  "T2"   },
  { MS_WORK,         0,   MS_EX_TIME,    1500,      200,   8000,  2,  "WORK2"},
  { MS_TURN,      +180,   MS_EX_TURN,       0,      200,   9000,  0,  "U180" },
  { MS_TRACK,        0,   MS_EX_JUNC,       0,      400,   9000,  0,  "BACK2"},
  { MS_TURN,  MS_TURN_MEMO, MS_EX_TURN,     0,      200,   6000,  0,  "M90_2"},
/* -- 直行 -> 岔路口拐弯进 3 号房间 --------------------------------------- */
  { MS_CROSS,        1,   MS_EX_NONE,       0,      400,  12000,  0,  "J4"   },
  { MS_TRACK,        0,   MS_EX_ROOM,       0,      400,  12000,  0,  "T3"   },
  { MS_WORK,         0,   MS_EX_TIME,    1500,      200,   8000,  3,  "WORK3"},
  { MS_TURN,      +180,   MS_EX_TURN,       0,      200,   9000,  0,  "U180" },
  { MS_TRACK,        0,   MS_EX_JUNC,       0,      400,   9000,  0,  "BACK3"},
  { MS_TURN,  MS_TURN_MEMO, MS_EX_TURN,     0,      200,   6000,  0,  "M90_3"},
/* -- 直行 -> 普通弯(右拐)进 4 号房间 (普通弯不写行, MS_TRACK 自己转) ------ */
  { MS_TRACK,        0,   MS_EX_ROOM,       0,      400,  12000,  0,  "T4"   },
  { MS_WORK,         0,   MS_EX_TIME,    1500,      200,   8000,  4,  "WORK4"},
/* -- 4 号房间测完掉头; 出门那个口是【普通弯右拐】(主路到这就没了, 不是 T 字口)
 *    -> 自动转, 所以这里【不写 MEMO 行】--------------------------------- */
  { MS_TURN,      +180,   MS_EX_TURN,       0,      200,   9000,  0,  "U180" },
/* -- 沿走过的这条路回去: 直行经过两个岔路口 (都直行穿过) ----------------- */
  { MS_CROSS,        0,   MS_EX_NONE,       0,      400,  12000,  0,  "J5"   },
  { MS_CROSS,        0,   MS_EX_NONE,       0,      400,  12000,  0,  "J6"   },
/* -- T 字路口左拐 (左 = -90; 走转弯序列: 前移 250ms + 原地转 + 找线) ------ */
  { MS_TURN,       -90,   MS_EX_TURN,       0,      200,   9000,  0,  "L90"  },
/* -- 直行 (中间一个普通弯左拐) -> 岔路口拐弯 ------------------------------ */
  { MS_CROSS,        1,   MS_EX_NONE,       0,      400,  12000,  0,  "J7"   },
/* -- 直行 (中间一个普通弯左拐) -> 模拟充电区 ----------------------------- */
  { MS_TRACK,        0,   MS_EX_ROOM,       0,      400,  15000,  0,  "CHG"  },
/* -- 充电区停 5 秒 ------------------------------------------------------- */
  { MS_WAIT,         0,   MS_EX_TIME,    5000,      300,   8000,  0,  "CHG5S"},
/* -- 原地掉头; 出门那个普通弯左拐自动转 -> 回主路 -> 直行 -> 停车场 ------ */
  { MS_TURN,      +180,   MS_EX_TURN,       0,      200,   9000,  0,  "U180" },
  { MS_TRACK,        0,   MS_EX_ROOM,       0,      400,  15000,  0,  "PARK" },
/* -- 收工 --------------------------------------------------------------- */
  { MS_END,          0,   MS_EX_NONE,       0,        0,      0,  0,  "END"  },
};

#define ROUTE_N_ALL  ((int)(sizeof(route) / sizeof(route[0])))

/* ---- 台架分段调试开关 (不动上面的表就能一段一段验) ----------------------
 * 0    = 跑完整张表 (31 行)
 * N>0  = 只跑表里前 N 行 (第 0 行 ~ 第 N-1 行), 之后收工停车。对照上面 route[] 的行号:
 *          1 = J1    循迹 + 直行穿过第 1 个岔路口
 *          2 = +J2   第 2 个岔路口(直行穿过)
 *          3 = +T1   循迹到 1 号房间口
 *          4 = +WORK1
 *          5 = +U180 原地掉头
 *          6 = +BACK1 沿支线回上一个路口
 *          7 = +M90_1 转回主路        <- 进完 1 号房间并回到主路
 *         13 = +J3~M90_2 (2 号房间走完) | 19 = +J4~M90_3 (3 号房间走完)
 *         23 = +T4~U180  (4 号房间走完) | 26 = +J5~L90 (回程两个岔路口 + T字口左拐)
 *         28 = +J7~CHG5S (岔路口 + 充电区 + 停5s) | 31 = +U180~END (整表)
 *        每验通一段把 N 加 1; 也可以用 Keil Misc Controls 加 -DMISSION_DEBUG_STEPS=2 */
#ifndef MISSION_DEBUG_STEPS
#define MISSION_DEBUG_STEPS   0      /* ★ 0 = 跑【整张表】(31 行, 一路到停车场收工)
                                      *   N>0 = 只跑前 N 行 (一段段验的时候用, 见上面那张对照表)
                                      *   还想分段验就把 0 改回 7/13/19/23/26/28 之一 */
#endif
#define ROUTE_N   (((MISSION_DEBUG_STEPS) > 0 && (MISSION_DEBUG_STEPS) < ROUTE_N_ALL) \
                   ? (MISSION_DEBUG_STEPS) : ROUTE_N_ALL)

#define SUB_RUN   0
#define SUB_STOP  1
#define SUB_DONE  2

static int      ms_active       = 0;
static int      ms_step         = 0;
static uint8_t  ms_sub          = SUB_DONE;
static uint32_t ms_sub_t0       = 0;    /* 本子状态(停车段)开始时刻 */
static uint32_t ms_step_t0      = 0;    /* 本步开始时刻 (计时/超时用) */
static uint8_t  ms_turn_started = 0;    /* 本步的 IMU 转向已启动 */
static uint8_t  ms_work_entered = 0;    /* 本步的 MS_WORK 钩子已调过 */
static char     ms_status[24];

/* ---- 路口/转弯 (2026-10-06) ---- */
static uint8_t  ms_auto_turn        = 0;   /* 普通弯: 正在"前移+原地转" (转完回到本步继续) */
static uint8_t  ms_turn_seq_on      = 0;   /* MS_TURN 步: 正在跑统一转弯序列 (前移+转+找线) */
static uint8_t  ms_cross_phase      = 0;   /* MS_CROSS 内部相位: 0=循迹等岔路 1=前移+转 2=盲走穿口 */
static uint32_t ms_cross_t0         = 0;   /* 盲走穿口起点 */
static uint8_t  ms_corner_done      = 0;   /* 本步里已经自动转过一个普通弯 (MS_CROSS 判断用) */
static int      ms_last_branch_side = 0;   /* 最近一次岔路的分支侧 (+1右/-1左): 回主路那步要用 */
static uint8_t  ms_busy             = 0;   /* 本拍动作没做完(自动转弯/前移中) -> 别判退出条件 */

/* ---- 转弯序列的状态 (定义放这里, 因为 Mission_Init/Start 里要复位它) ---- */
#define TS_IDLE   0
#define TS_ADV    1
#define TS_TURN   2
#define TS_FIND_F 3                    /* (1) 转完直进找线 (先做, 最便宜) */
#define TS_ARC_A  4                    /* (2) 首选侧画弧找线 (前进 + 侧偏) */
#define TS_ARC_B  5                    /* (3) 镜像侧画弧 */
#define TS_FIND_B 6                    /* (4) 直退找线 (覆盖"车开过头, 线在车后") */
#define TS_SCAN   7                    /* (5) 原地左右扫描 (只覆盖横向, 不用挪位置) */
#define TS_BACK   8                    /* 扫描完: 转回"线最近"的那一步再收工 */
static uint8_t  ts_state     = TS_IDLE;
static uint32_t ts_t0        = 0;      /* 前移起点 */
static int      ts_adv_ms    = 0;      /* 本次前移时长 (MS_TURN 用 MS_TURN_ADV_MS, 0 = 不前移) */
static uint8_t  ts_forced_deg = 0;     /* 1 = 角度是外部给的 (MS_TURN 步) -> 串口一定打报告 */
static uint32_t ts_t_turn    = 0;      /* 起转/扫描每一步的时刻 (+停稳计时) */
static int      ts_deg       = 0;      /* 本次转向的 ImuTurn 角度 */
static uint32_t ts_t_find    = 0;      /* 找线阶段计时起点 */
static uint8_t  ts_adv_gone  = 0;      /* 前移期间"主线整段消失"的连续帧数 */
static uint8_t  ts_is_corner = 0;      /* 1 = 前移读线确认这是直角弯 (不是岔路) */
static int      ts_arc_dir   = 1;      /* 当前弧线往哪边 */
static uint8_t  ts_saw_line  = 0;      /* 转弯过程中看到过线? */
static int      ts_last_dev  = 0;      /* 最后看到线时, 线偏阵列中心多少 (x10 路, 正 = 偏右) */
static int      ts_last_deg  = 0;      /* 看到线时转过多少度 */
static uint8_t  ts_found_ph  = 0;      /* 靠哪一步找到线: 0=转到位 1=直进 2=弧A 3=弧B 4=直退 5=扫描 9=失败 */
static int      ts_scan_k    = 0;      /* 扫描已走步数 */
static int      ts_scan_best  = -99;   /* 扫描中"线离阵列中心最近"那一步的相对角度 */
static int      ts_scan_bestd = 9999;  /* 那一步的偏差 (x10 路) */
static uint8_t  ms_fail      = 0;      /* 0=正常 1=找线失败(NOLINE) 2=转弯序列超时/起不来(TURN fail) */

/*---------------------------------------------------------------------------
 * 弱钩子: 默认空实现; 在 empty.c 里同名再写一遍就覆盖了 (不用改本文件)
 *-------------------------------------------------------------------------*/
__attribute__((weak)) void Mission_OnWork(int room)      { (void)room; }
__attribute__((weak)) void Mission_OnStepEnter(int step) { (void)step; }
__attribute__((weak)) void Mission_OnDone(void)          { }

/*---------------------------------------------------------------------------
 * 内部
 *-------------------------------------------------------------------------*/
static void ms_sync_now(uint32_t now)
{
    ms_sub_t0  = now;
    ms_step_t0 = now;
}

static void ms_finish(void)
{
    ms_active = 0;
    ms_sub    = SUB_DONE;
    Track_Stop();
    Track_Detect(1);                /* * 收工/急停: 保证检测是打开的 (转向中途按 S2 也不会把判定关死) */
    Mission_OnDone();
}

/* 转弯序列被判死/IMU 起不来时的统一收工: 停车 + 标记 + 任务结束 (屏显 TURN fail) */
static void ms_fail_seq(void)
{
    ts_state = TS_IDLE;
    ms_fail  = 2;
    ms_finish();
}

/* 进入"步间停车" */
static void ms_enter_stop(uint32_t now)
{
    ms_sub    = SUB_STOP;
    ms_sub_t0 = now;
    Track_Stop();

    /* [A] 步间停车 = 刚做完一个动作(多半是转弯): 检测重新打开 + 再屏蔽 TRACK_SUPPRESS_MS。
     *     原因: 这时车多半还在路口边上/线在阵列边缘, 立刻判会判出假岔路 -> 连转 (用户实测)。 */
    Track_Detect(1);
    Track_Suppress(TRACK_SUPPRESS_MS);
}

/* 进入下一步 */
static void ms_next_step(uint32_t now)
{
    ms_step++;
    if (ms_step >= ROUTE_N) { ms_finish(); return; }

    ms_sub          = SUB_RUN;
    ms_turn_started = 0;
    ms_work_entered = 0;
    ms_cross_phase  = 0;             /* 岔路口相位 / 普通弯标记 每步都从头开始 */
    ms_corner_done  = 0;

    /* 停车期间(车没动)攒下的路口/房间边沿事件丢掉, 否则下一步一进 RUN 就立刻误触发 */
    (void)Track_TakeCross();
    (void)Track_TakeRoom();
    (void)Track_TakeBranch();       /* 岔路 / 普通弯 事件也要丢掉 (2026-10-06) */
    (void)Track_TakeCorner();

    ms_sync_now(now);
    Mission_OnStepEnter(ms_step);
}

/* 退出条件成立? (每拍在 SUB_RUN 里问一次) */
static int ms_exit_met(const MissionStep *s, uint32_t now)
{
    switch (s->exit)
    {
    case MS_EX_CROSS: return Track_TakeCross();
    case MS_EX_ROOM:  return Track_TakeRoom();
    case MS_EX_JUNC:  return Track_TakeRoom();   /* 到 T 字口/主线路口: 判据同"大面积黑" */
    case MS_EX_TIME:  return ((now - ms_step_t0) >= (uint32_t)s->exit_arg);
    case MS_EX_TURN:  return (ms_turn_started && !ImuTurn_IsBusy());
    case MS_EX_LINE:  return (((~track_bits) & (uint8_t)s->exit_arg) != 0);  /* 掩码里任一路压线 */
    default:          return 0;
    }
}

/*---------------------------------------------------------------------------
 * 对外接口
 *-------------------------------------------------------------------------*/
void Mission_Init(void)
{
    ms_active           = 0;
    ms_step             = 0;
    ms_sub              = SUB_DONE;
    ms_turn_started     = 0;
    ms_work_entered     = 0;
    ms_auto_turn        = 0;
    ms_turn_seq_on      = 0;
    ms_cross_phase      = 0;
    ms_corner_done      = 0;
    ms_last_branch_side = 0;
    ms_busy             = 0;
    ts_state            = TS_IDLE;
    ms_fail             = 0;
    ms_sync_now(Get_Time());
    Track_Detect(1);                /* 上电保证判定是打开的 */
}

void Mission_Start(void)
{
    ms_active           = 1;
    ms_step             = 0;
    ms_sub              = SUB_RUN;
    ms_turn_started     = 0;
    ms_work_entered     = 0;
    ms_auto_turn        = 0;
    ms_turn_seq_on      = 0;
    ms_cross_phase      = 0;
    ms_corner_done      = 0;
    ms_last_branch_side = 0;
    ms_busy             = 0;
    ts_state            = TS_IDLE;
    ms_fail             = 0;
    ms_sync_now(Get_Time());
    Track_Detect(1);                /* 打开岔路/普通弯判定 */
    Track_Suppress(TRACK_SUPPRESS_MS);
    Track_Reset();                  /* 清循迹的丢线计时/PD 历史 (+ XunJi 里清路口/岔路事件) */
    Mission_OnStepEnter(0);
}

void Mission_Stop(void)
{
    ImuTurn_Stop();                 /* 转向中就先停它 */
    ms_finish();
}

int Mission_IsActive(void) { return ms_active; }
int Mission_GetStep(void)  { return ms_step; }
char Mission_GetSub(void)
{
    if (!ms_active)             return 'D';
    if (ms_sub == SUB_STOP)     return 'S';
    if (ts_state == TS_FIND_F || ts_state == TS_FIND_B) return 'F';   /* 直进/直退找线 */
    if (ts_state == TS_ARC_A  || ts_state == TS_ARC_B)  return 'A';   /* 弧线找线 */
    if (ts_state == TS_SCAN   || ts_state == TS_BACK)   return 'X';   /* 原地扫 */
    return 'R';
}

int Mission_GetFail(void) { return ms_fail; }   /* 1 = 上一次任务因为找不到线停了 (屏显 NOLINE) */

/* ---- 上一次转弯的观测值 (标定用, 屏/串口看) ---- */
int Mission_GetTurnDev(void)     { return ts_last_dev; }     /* 看到线时偏阵列中心多少 (x10 路, 正=偏右) */
int Mission_GetTurnDegSeen(void) { return ts_last_deg; }     /* 看到线时转过多少度 */
int Mission_GetTurnFound(void)   { return ts_found_ph; }     /* 靠哪一步找到线: 0=转到位 1=直进 2=弧A 3=弧B 4=直退 5=扫描 9=失败 */
int Mission_GetTurnAdvGone(void) { return ts_is_corner; }    /* 1 = 前移时就发现"主线没了" -> 这个口其实是直角弯 */

const char *Mission_GetStepName(void)
{
    if (ms_step >= ROUTE_N) return "----";
    return route[ms_step].name;
}

const char *Mission_GetStatus(void)
{
    if (!ms_active) { sprintf(ms_status, "IDLE"); return ms_status; }
    sprintf(ms_status, "M%02d %-4s %c", ms_step, Mission_GetStepName(), Mission_GetSub());
    return ms_status;
}

int Mission_GetCrossCnt(void) { return track_cross_cnt; }

/*---------------------------------------------------------------------------
 * 5ms 推进
 *-------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------
 * 转弯序列: 盲走前移 MS_ADV_MS -> 原地转 90 度 (方向由传感器判)
 *   ★ 方向: side = +1 右转 / -1 左转 (XunJi 的岔路/普通弯判定给的)
 *   ★ 为什么要前移: 见文件顶部 MS_ADV_MS 的说明 (8 路灰度装在小车最前面)
 *   ★ 前移期间直接写 Motor_SetLeftPWM/RightPWM (盲走), 不经过 track_output 的观察模式闸门
 *---------------------------------------------------------------------------*/
static void turn_seq_start(int side, int deg_fixed, int adv_ms)
{
    /* deg_fixed != 0: 直接用这个角度 (MS_TURN 的 arg / MEMO 换算值);
     * deg_fixed == 0: 由传感器判出来的侧向映射 (岔路/普通弯) */
    /* ★ 用户定的规则 (2026-10-07): 【掉头 (180) 不前移, 直接原地转】;
     *   【从房间支线回到上一个路口 (T 字口, 90 度) 才要前移 MS_TURN_ADV_MS】。
     *   两个口在传感器看来都是"大面积黑 T 字口", 但一个有前移一个没有 —— 区别在转的角度:
     *     - 180 掉头: 绕几何中心转 -> 传感器从"车前 L"变到"车后 L", 线顺着航向走,
     *       前后挪 2L 不影响压线; 前移反而把车带出工作位置/顶到房间尽头 -> 掉头做不成
     *       (2026-10-07 上板实测: 掉头前多那 250ms, 掉头就没了, 直接进找线兜底)。
     *     - 90 度转弯 (回主路): 必须补这段, 否则转完传感器落在线外 (见文件头 MS_ADV_MS)。 */
    if ((deg_fixed != 0) && ((deg_fixed >= 180) || (deg_fixed <= -180))) adv_ms = 0;
    ts_adv_ms    = (adv_ms > 0) ? adv_ms : 0;      /* 0 = 不前移, 立刻起转 */
    ts_forced_deg = (deg_fixed != 0) ? 1 : 0;      /* MS_TURN 步: 串口打报告 (排查掉头用) */
    ts_deg       = (deg_fixed != 0) ? deg_fixed
                                    : ((side > 0) ? MS_DEG_RIGHT : MS_DEG_LEFT);
    Track_Detect(0);                       /* [D] 前移+原地转+找线 全过程关掉岔路/普通弯判定 */
    ts_adv_gone  = 0;
    ts_is_corner = 0;
    ts_arc_dir   = MS_ARC_FIRST_SIDE;
    ts_saw_line  = 0;
    ts_last_dev  = 0;
    ts_last_deg  = 0;
    ts_found_ph  = 0;
    ts_scan_k    = 0;
    ts_scan_best = -99;
    ts_scan_bestd = 9999;
    ts_state     = TS_ADV;
    ts_t0        = Get_Time();
    Track_Stop();                          /* 先停稳, 前移距离才好重复 */
}

/* 扫描到第 ts_scan_k 步时的【相对起始航向】角度 (用于"回到最近的那一步") */
static int ts_scan_rel(void)
{
    int half = MS_SCAN_DEG / MS_SCAN_STEP;
    int k    = (ts_scan_k > 2*half) ? 2*half : ts_scan_k;

    return (k <= half) ? (k * MS_SCAN_STEP) : ((2*half - k) * MS_SCAN_STEP);
}

/* 转弯收工时的观测上报: 只在"异常/要看落点"时打串口
 *   (printf 走 LORA UART3 9600 波特, 一行 ~25 字符 = 26ms 阻塞; 正常转弯不打, 免得卡控制周期) */
static void ts_report(void)
{
    /* 正常 90 度岔路/普通弯 (靠传感器出口收工) 不打, 免得 9600 波特拖慢控制周期;
     * MS_TURN 步 (掉头 / 回主路) 【一定打】 —— 这一行是排查掉头的唯一凭据:
     *   d = 目标角  r = 结束原因 (0=到位 1=被停 2=IMU掉线 3=超时 4=越走越远/被顶住)
     *   g = 前移时主线就没了(其实是直角弯)  s = 转过中看到线  y = 看到线时线偏阵列中心
     *   f = 靠哪一步找到线 (0=转到位 1=直进 2=弧A 3=弧B 4=直退 5=扫描 9=失败) */
    if (!ts_forced_deg && !ts_is_corner && (ts_found_ph == 0)) return;

    printf("TURN d%+d r%d g%d s%d y%+d f%d\r\n",
           ts_deg, ImuTurn_GetDoneReason(), ts_is_corner, ts_saw_line, ts_last_dev, ts_found_ph);
}

/* 找线各阶段共用: 读到线就收工 (返回 1) */
static int ts_find_exit(int phase)
{
    Track_ReadOnce();
    if (!Track_LineFound()) return 0;
    Track_Stop();
    ts_found_ph = (uint8_t)phase;
    ts_report();
    ts_state = TS_IDLE;
    return 1;
}

/* 返回 1 = 整段 (前移 + 转向 + 找线) 做完 */
static int turn_seq_update(void)
{
    /* 总看门狗: 整段序列 (前移+转+找线) 跑太久 = 某一步卡住了 (IMU 没数据/一直忙) ->
     * 停车报警, 别让车带着"没完成的转向"往下走 */
    if ((Get_Time() - ts_t0) >= (uint32_t)MS_SEQ_MAX_MS)
    {
        Track_Stop();
        printf("TURNSEQ timeout step %d\r\n", ms_step);
        ms_fail_seq();
        return 1;
    }

    switch (ts_state)
    {
    case TS_ADV:                                            /* (0) 盲走直行: 补上传感器前伸的距离 */
        /* [b] 前移期间也读线: 主线还在? 关键判据 ——
         *   岔路: 主线继续 -> 阵列中心一直有线;  直角弯: 主线拐走了 -> 阵列会整段变白。
         *   所以"前移中就白了好几天"= 这个口其实是直角弯 (记下来, 屏/串口能看到语义纠正)。 */
        if (MS_ADV_READ)
        {
            Track_ReadOnce();
            if (track_nblack == 0) { if (ts_adv_gone < 250) ts_adv_gone++; }
            else                   { ts_adv_gone = 0; ts_is_corner = 0; }
            if (ts_adv_gone >= MS_ADV_GONE_FRAMES) ts_is_corner = 1;
        }
        if ((Get_Time() - ts_t0) >= (uint32_t)(ts_adv_ms + MS_ADV_FAIL_MS))
        {
            /* IMU 一直起不来 (没数据 / 一直被占): 再等就是白冲这 300ms 还在往前开 -> 停车报警 */
            Track_Stop();
            printf("TURNSTALL step %d\r\n", ms_step);
            ms_fail_seq();
            return 1;
        }
        if ((Get_Time() - ts_t0) >= (uint32_t)ts_adv_ms)   /* ts_adv_ms = 0 -> 立刻起转 (不前移) */
        {
            Track_Stop();
            if (!ImuTurn_IsBusy() && (ImuTurn_TurnDeg(ts_deg, 0) == 0))
            {
                ts_state  = TS_TURN;
                ts_t_turn = Get_Time();
                if (ts_forced_deg)                        /* MS_TURN 步: 记下"转向真的发出去了" */
                    printf("TGO %+d a%d\r\n", ts_deg, ts_adv_ms);
            }
            /* 启动失败 (IMU 正忙/没数据): 下一拍再试;
             * 宽限 MS_ADV_FAIL_MS 用完还起不来 -> 上面那个分支停车报警 (MS_TURN 步的 timeout_ms 不兜这个) */
        }
        else
        {
            Motor_SetLeftPWM(MS_BLIND_SPEED);
            Motor_SetRightPWM(MS_BLIND_SPEED);
        }
        return 0;

    case TS_TURN:                                           /* (2) 原地转: 出口优先用传感器 (c) */
        {
            int diff = (int)ImuTurn_GetDiff();               /* 剩余角度 (带符号) */
            int ad   = (diff < 0) ? -diff : diff;
            int prog = ((ts_deg < 0) ? -ts_deg : ts_deg) - ad;   /* 已经转过多少度 */

            if (prog < 0) prog = 0;

            /* [e] 每拍读线 + 记下(转过多少度, 线在阵列上的位置): 标定/诊断用
             *   (线位置 x10 路, 正 = 偏阵列右端; 屏幕/串口能看) */
            Track_ReadOnce();
            if (track_nblack > 0)
            {
                int i, sum = 0, n = 0;
                for (i = 0; i < 8; i++) if (!((track_bits >> i) & 1)) { sum += i; n++; }
                if (n)
                {
                    ts_saw_line = 1;
                    ts_last_dev = sum * 10 / n - 35;
                    ts_last_deg = prog;
                }
            }

            /* [c] *预防主力*: 细线回到阵列正中 = 车已经转到位 + 线在车轴正下方 -> 立刻收工。
             *   门槛 = max(MS_TURN_SENSOR_DEG, |目标角| - MS_TURN_SENSOR_TAIL):
             *     90 度转弯 -> 60 度 (老行为不变);  180 度掉头 -> 155 度 (只在快到终点时才认)。
             *   转过门槛前被"正中"骗到就是掉头被掐断 (见 MS_TURN_SENSOR_TAIL 说明)。 */
            {
                int need = (ts_deg < 0) ? -ts_deg : ts_deg;

                need -= MS_TURN_SENSOR_TAIL;
                if (need < MS_TURN_SENSOR_DEG) need = MS_TURN_SENSOR_DEG;

                if (MS_TURN_SENSOR_EXIT && (prog >= need) && Track_LineCentered())
                {
                    ImuTurn_Stop();
                    ts_found_ph = 0;
                    ts_report();
                    ts_state = TS_IDLE;
                    return 1;
                }
            }

            if (!ImuTurn_IsBusy())                           /* IMU 到角: 正常收工 -> 顺手试找线 */
            {
                ts_report();
                ts_state  = TS_FIND_F;
                ts_t_find = Get_Time();
            }
            return 0;
        }

    case TS_FIND_F:                                          /* (1) 直进找线 (最便宜, 先试) */
        if (ts_find_exit(1)) return 1;
        if ((Get_Time() - ts_t_find) >= (uint32_t)MS_FIND_MS_SHORT)
        {
            Track_Stop();
            ts_state  = TS_ARC_A;
            ts_t_find = Get_Time();
            return 0;
        }
        Motor_SetLeftPWM(MS_FIND_SPEED);
        Motor_SetRightPWM(MS_FIND_SPEED);
        return 0;

    case TS_ARC_A:                                           /* (2)(3) 弧线找线: 前进 + 侧偏 */
    case TS_ARC_B:
        if (ts_find_exit(ts_state == TS_ARC_A ? 2 : 3)) return 1;
        if ((Get_Time() - ts_t_find) >= (uint32_t)MS_ARC_MS)
        {
            Track_Stop();
            if (ts_state == TS_ARC_A)                        /* 首选侧走完 -> 镜像走另一侧 */
            {
                ts_state   = TS_ARC_B;
                ts_arc_dir = -ts_arc_dir;
            }
            else
            {
                ts_state = TS_FIND_B;                        /* 两侧弧都走完 -> 退回来找 */
            }
            ts_t_find = Get_Time();
            return 0;
        }
        /* 左轮快 = 右转 (与循迹层同一套符号); 弯度 = MS_ARC_BIAS
         * 横向覆盖 = R*(1-cos φ): 差速太小(弧太直)横向只有 1cm 级, 要覆盖 3~4cm 得让航向扫 40~60 度 */
        if (ts_arc_dir > 0)
        {
            Motor_SetLeftPWM (MS_ARC_SPEED + MS_ARC_BIAS);
            Motor_SetRightPWM(MS_ARC_SPEED - MS_ARC_BIAS);
        }
        else
        {
            Motor_SetLeftPWM (MS_ARC_SPEED - MS_ARC_BIAS);
            Motor_SetRightPWM(MS_ARC_SPEED + MS_ARC_BIAS);
        }
        return 0;

    case TS_FIND_B:                                          /* (4) 直退找线 (线在车后) */
        if (ts_find_exit(4)) return 1;
        if ((Get_Time() - ts_t_find) >= (uint32_t)MS_FIND_MS)
        {
            Track_Stop();
            ts_state  = TS_SCAN;
            ts_scan_k = 0;
            ts_t_turn = Get_Time();
            return 0;
        }
        Motor_SetLeftPWM (-MS_FIND_SPEED);
        Motor_SetRightPWM(-MS_FIND_SPEED);
        return 0;

    case TS_SCAN:                                            /* (5) 原地左右扫描 (最后兜底) */
        if (ImuTurn_IsBusy()) return 0;
        if ((Get_Time() - ts_t_turn) < (uint32_t)MS_SCAN_SETTLE) return 0;   /* 每步停稳一下再读 */
        if (ts_find_exit(5)) return 1;

        /* 记下"线离阵列中心最近"的那一步 (扫完还找不到时退回到这里) */
        if (track_nblack > 0)
        {
            int i, sum = 0, n = 0, d;
            for (i = 0; i < 8; i++) if (!((track_bits >> i) & 1)) { sum += i; n++; }
            if (n)
            {
                d = sum * 10 / n - 35;
                if (d < 0) d = -d;
                if (d < ts_scan_bestd) { ts_scan_bestd = d; ts_scan_best = ts_scan_rel(); }
            }
        }

        {
            int half = MS_SCAN_DEG / MS_SCAN_STEP;
            int dir;

            if      (ts_scan_k <  half) dir = +1;            /* 先往一侧扫 */
            else if (ts_scan_k < 2*half) dir = -1;           /* 再扫回来, 过中线到另一侧 */
            else
            {
                Track_Stop();
                if (ts_scan_best > -90)                      /* 见过线: 转回"线离中心最近"的那一步 */
                {
                    int back = ts_scan_best - ts_scan_rel();

                    if ((back != 0) && (ImuTurn_TurnDeg(back, 0) == 0))
                    {
                        ts_state  = TS_BACK;
                        ts_t_turn = Get_Time();
                        return 0;
                    }
                    ts_state = TS_IDLE;
                    return 1;
                }
                /* 一次都没见到线: 停车报警 (不让它继续盲冲) */
                ts_state = TS_IDLE;
                ms_fail  = 1;
                printf("NOLINE step %d\r\n", ms_step);
                ts_report();
                ms_finish();
                return 1;
            }
            ts_scan_k++;
            ImuTurn_TurnDeg(dir * MS_SCAN_STEP, 0);
            ts_t_turn = Get_Time();
        }
        return 0;

    case TS_BACK:                                            /* (6) 转回线最近的那一步, 转完收工 */
        if (!ImuTurn_IsBusy()) { ts_state = TS_IDLE; return 1; }
        return 0;

    default:
        return 1;
    }
}

/*---------------------------------------------------------------------------
 * 循迹一拍 (+ 普通弯自动转) —— MS_TRACK 和 MS_CROSS 的"循迹段"共用
 *   返回 1 = 本拍在自动转弯/前移中 (别去判退出条件)
 *   普通弯(主线自己拐走了)不在路线表里: 这里自己"前移 + 原地转 90", 转完接着循迹
 *---------------------------------------------------------------------------*/
static int ms_do_track(void)
{
    if (ms_auto_turn)
    {
        if (turn_seq_update())
        {
            ms_auto_turn   = 0;
            ms_corner_done = 1;
            Track_Detect(1);                   /* [D] 转完重新打开判定 */
            Track_Suppress(TRACK_SUPPRESS_MS); /* [A] 紧接着屏蔽一段时间 (找线过程别乱判) */
            Track_Reset();                     /* 转完清循迹状态 (丢线计时/PD 历史) */
        }
        return 1;
    }

    Track_Run();

    if (Track_TakeCorner())                    /* 普通弯: 主线自己拐走了, 车自己转 */
    {
        ms_auto_turn = 1;
        turn_seq_start(track_corner_side, 0, MS_ADV_MS);
        return 1;
    }
    return 0;
}

void Mission_Run(void)
{
    const MissionStep *s;
    uint32_t now;

    if (!ms_active) return;

    now = Get_Time();
    s   = &route[ms_step];

    /* ---- 收工步 ---- */
    if (s->act == MS_END) { ms_finish(); return; }

    switch (ms_sub)
    {
    /* ================= 执行本步 ================= */
    case SUB_RUN:
        ms_busy = 0;

        /* 兜底超时: 条件一直不成立也往下走, 防止卡死在一步
         * (自动转弯/前移中不判超时, 否则一转就 500ms+ 会被误判) */
        if (!ms_auto_turn && (ms_cross_phase == 0) &&
            (s->timeout_ms > 0) && ((now - ms_step_t0) > (uint32_t)s->timeout_ms))
        {
            ms_enter_stop(now);
            return;
        }

        switch (s->act)
        {
        case MS_TRACK:
            ms_busy = ms_do_track();                      /* 循迹 + 普通弯自动转 */
            break;

        case MS_CROSS:                                    /* 岔路口: 循迹到岔路, 再 不转(盲走穿口) / 转(前移+自判方向) */
            if (ms_cross_phase == 0)
            {
                ms_busy = ms_do_track();
                if (!ms_busy && Track_TakeBranch())       /* 岔路确认: 主线还在, 该决定走哪边了 */
                {
                    ms_last_branch_side = track_branch_side;   /* 记住方向 (从房间支线回来那步要用) */
                    if (s->arg == 0)                      /* 不转: 盲走直行穿过去 */
                    {
                        ms_cross_phase = 2;
                        ms_cross_t0    = now;
                    }
                    else                                  /* 要转: 前移 + 原地转 90 (方向自判) */
                    {
                        ms_cross_phase = 1;
                        turn_seq_start(track_branch_side, 0, MS_ADV_MS);
                    }
                }
                else if (ms_corner_done)                  /* 这一步的"路口"其实是普通弯: 已经自动转过去了 */
                {
                    ms_cross_phase = 0;
                    ms_enter_stop(now);
                    return;
                }
            }
            else if (ms_cross_phase == 1)                 /* 前移 + 转向序列 */
            {
                ms_busy = 1;
                if (turn_seq_update())
                {
                    ms_cross_phase = 0;
                    ms_enter_stop(now);
                    return;
                }
            }
            else                                          /* 盲走穿口 */
            {
                ms_busy = 1;
                if ((now - ms_cross_t0) >= (uint32_t)MS_BLIND_THRU_MS)
                {
                    ms_cross_phase = 0;
                    ms_enter_stop(now);
                    return;
                }
                Motor_SetLeftPWM(MS_BLIND_SPEED);
                Motor_SetRightPWM(MS_BLIND_SPEED);
            }
            break;

        case MS_BLIND:                                    /* 盲走直行: 不看线 */
            {
                int p = (s->arg > 0) ? (int)s->arg : MS_BLIND_SPEED;
                Motor_SetLeftPWM(p);
                Motor_SetRightPWM(p);
            }
            break;

        case MS_TURN:                                     /* 定角转 (IMU): arg = 角度 或 MS_TURN_MEMO
                                                           * ★ 两种走法 (2026-10-07 用户定的规则, 不要再混):
                                                           *   (a) 180 度掉头 (|arg| >= 180):
                                                           *       【原样的直接原地转】—— 不前移、不进转弯序列、
                                                           *       不找线。这就是最早能跑的那一版 (用户实测掉头正常);
                                                           *       掉头前那 250ms 前移会把车带出工作位置 -> 掉头做不成。
                                                           *   (b) 90 度 (MS_TURN_MEMO, 从房间支线回上一个路口):
                                                           *       【走转弯序列】—— 这个口(也是 T 字口)必须先
                                                           *       盲走 MS_TURN_ADV_MS 再原地转 90 (用户明确要求),
                                                           *       转完还会找线兜底。 */
            /* ★ 判据必须排掉 MS_TURN_MEMO (-2000): 它也 <= -180, 漏掉就会把 -2000 当角度
             *   传给 ImuTurn (测试台抓到过: TurnDeg(-2000))。 */
            if ((s->arg != MS_TURN_MEMO) &&
                ((s->arg >= 180) || (s->arg <= -180)))     /* ---- (a) 掉头: 直接转, 别的什么都不做 ---- */
            {
                if (!ms_turn_started)
                {
                    if (!ImuTurn_IsBusy() && (ImuTurn_TurnDeg((int)s->arg, 0) == 0))
                    {
                        ms_turn_started = 1;
                        Track_Reset();
                        printf("TGO %+d a-1\r\n", (int)s->arg);   /* a-1 = 直接转 (无前移/无序列) */
                    }
                    /* 启动失败 (IMU 正忙/没数据): 下一拍再试; 由本步 timeout_ms 兜底 (原样) */
                }
                break;
            }

            if (!ms_turn_seq_on)
            {
                int deg = (int)s->arg;

                if (deg == MS_TURN_MEMO)                      /* 从支线回主路: 用记住的岔路方向 */
                {
                    int sd = ms_last_branch_side ? ms_last_branch_side : 1;
                    deg = (sd > 0) ? MS_DEG_RIGHT : MS_DEG_LEFT;
                    /* 注: 进支线转 90 + 掉头 180 + 再转 90 = 转回原航向, 所以方向和进来时【同侧】 */
                }

                ms_turn_started = 1;                          /* MS_EX_TURN 还要用它 */
                ms_turn_seq_on  = 1;
                ms_auto_turn    = 1;                          /* 复用: 序列进行中 -> 本步不判超时/退出条件 */
                turn_seq_start(0, deg, MS_TURN_ADV_MS);       /* 0 = 侧向不用 (角度已给定) */
                Track_Reset();
                return;
            }

            ms_busy = 1;                                      /* 序列没做完 -> 别判退出条件 */
            if (turn_seq_update())                            /* 整段 (前移 + 转 + 找线) 做完 */
            {
                ms_turn_seq_on = 0;
                ms_auto_turn   = 0;
                Track_Detect(1);                              /* 与 ms_do_track 收尾一致 */
                Track_Suppress(TRACK_SUPPRESS_MS);
                Track_Reset();
                /* 序列自己报错收工时 (NOLINE / TURN fail) ms_active 已经 0 -> 别再改子状态 */
                if (ms_active) ms_enter_stop(now);
            }
            return;

        case MS_WAIT:
            Track_Stop();
            break;

        case MS_WORK:                                     /* 工作环节: 停车 + 钩子 */
            Track_Stop();
            if (!ms_work_entered)
            {
                ms_work_entered = 1;
                ms_step_t0      = now;                    /* 工作计时从这里开始 */
                Mission_OnWork((int)s->room);
            }
            break;

        default:
            Track_Stop();
            break;
        }

        /* ---- 退出条件 ---- */
        if (!ms_busy && ms_exit_met(s, now)) ms_enter_stop(now);   /* 自动转弯/前移中不判 */
        return;

    /* ================= 步间停车 (以后插功能的地方) ================= */
    case SUB_STOP:
        Track_Stop();
        if (s->stop_ms == 0 || (now - ms_sub_t0) >= (uint32_t)s->stop_ms) ms_next_step(now);
        return;

    default:
        return;
    }
}
