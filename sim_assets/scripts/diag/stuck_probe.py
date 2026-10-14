#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实测 "撞上去之后无法重新规划、卡在原地不动" —— 判定卡住时到底卡在哪一环。

为什么"卡住"是可预期的 (代码链路):
  1) FSM checkCollisionCallback(): 发现轨迹与障碍相交时
       try planFromLocalTraj(true,false)
       成功 -> EXEC_TRAJ; 失败 -> 若 t_temp-t_cur < emergency_time_(1.0 s)
               则 EMERGENCY_STOP, 否则 REPLAN_TRAJ;
  2) EMERGENCY_STOP 里只有 (enable_fail_safe_ && odom_vel<0.1) 才回 GEN_NEW_TRAJ;
     enable_fail_safe_ 被 "Depth Lost!" 置 false, 只在**收到新目标**时重新武装
     -> 撞完若没人再给目标, 就永久悬停 (源码注释点名的 "stuck after a collision");
  3) 更硬的一层: 机体中心一旦落进**膨胀障碍格**, A* 的起点就是非法格
     (dyn_a_star 把被占/出图格视为 occupied) -> planFromLocalTraj 永远失败
     -> EMERGENCY_STOP <-> REPLAN_TRAJ 死循环, 速度 0, 位置不变;
  4) /drone_i_planning/pos_cmd 的 trajectory_flag 直接暴露这类状态:
       0 EMPTY, 1 READY, 3 COMPLETED, 4 ABORT, 5 ILLEGAL_START, 6 ILLEGAL_FINAL,
       7 IMPOSSIBLE。

对每个"卡住事件"(v_real < 0.05 m/s 持续 >= 1.5 s) 打印伴随证据:
  · d_inf / in_inflate: 机体中心到膨胀障碍格距离, 是否落在障碍格内
  · trajectory_id 在事件期间是否变化 (不变化 = 没有新轨迹产生)
  · trajectory_flag 取值集合 (含枚举翻译)
  · err_track = |pos_cmd - pos_true| (指令在远处而机体不动 => 指令有、飞不动)
  · d_obs: 到**原始**障碍图(未膨胀)的最近距离 (真撞上了, 还是只是被膨胀体困住)

用法 (先 source $TECH/sim_assets/env.sh; 仿真在跑; 复现"撞了之后卡住"):
  python3 stuck_probe.py 60 0              # 采样 60 s, drone_0, 阈值 1.5 s / 0.05 m/s
  python3 stuck_probe.py 60 0 3.0 0.08     # 自定义: 卡住判定 3.0 s, 速度阈 0.08

本工程实测注意 (2026-10-13):
  · occupancy 话题常常一帧点都不发 -> d_obs 恒为 inf, 用膨胀图 d_inf 判;
  · 机体在地面 (z<0.2 m) 时地面层/起飞点就在膨胀图里 -> 必然"被包住", 属假阳性,
    所以"被困"只在 z>=0.2 m 的帧里统计 (脚本已内置);
  · virtual_ceil_height 的虚天花板在膨胀图里是**一整层**(z≈2.0, 占满 xy) -> 别把
    "旁边有膨胀格"当成被围, d_inf 要看到柱身高度那一层的格才算;
   · 【口径修正】d_inf/在格内判定用 **odom(visual_slam, 与膨胀图同系)** 而不是 Gazebo 真值:
     真值是 model origin, 比 odom(雷达点)低约 mount_z = 0.20 m (drone_all.launch 里
     offset_z = mount_z) -> 用真值算 d_inf 系统性偏大, 会漏判"起点非法";
   · 【交叉验证】2026-10-14 实测: 机体卡在 (5.71,-6.65,0.75) 时,
     scene_check.py 从 formation.world 几何算得离 pillar_g072 柱面 **0.35 m**
     (正好等于 obstacles_inflation), 点云侧 d_inf = 0.011 m -> 两法一致,
     坐实"机体中心踩在膨胀格边界上"(停在这里就再也规划不出来);
   · 采样前后会对比 roslaunch 日志里 replan / Emergency stop / collision 的计数增量:
     "replan 猛涨而 traj_id 不变" = FSM 在拼命重试但每次都失败 (不是 FSM 停摆)。
"""

import os
import re
import sys
import time

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
ID = int(sys.argv[2]) if len(sys.argv) > 2 else 0
STUCK_T = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5   # 静止多久算卡住 [s]
V_TH = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05     # 静止速度阈 [m/s]
HALF_VOX = 0.06
FLAG = {0: 'EMPTY', 1: 'READY', 3: 'COMPLETED', 4: 'ABORT',
        5: 'ILLEGAL_START', 6: 'ILLEGAL_FINAL', 7: 'IMPOSSIBLE',
        -1: '无(该段没收到 pos_cmd)'}

# roslaunch 输出日志: 用来分辨 FSM 是"拼命重规划但每次都失败"还是"根本没在重试"
LOG_PATH = os.environ.get('PLANNER_LOG') or os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 '..', '..', 'logs', 'formation.log'))
LOG_PATS = {
    'replan': r'\[drone %d replan ' % ID,
    'try_replan': r'Try to replan a safe trajectory',
    'emergency': r'Emergency stop',
    'collision': r'current traj in collision',
    'depth_lost': r'Depth Lost',
    'near_goal': r'Close to goal',
}


def log_counts(nbytes=512 * 1024):
    """统计 planner 日志尾部窗口里各类事件条数 (日志可能上百 MB, 只看尾部)。"""
    try:
        sz = os.path.getsize(LOG_PATH)
        with open(LOG_PATH, 'rb') as f:
            f.seek(max(0, sz - nbytes))
            txt = f.read().decode('utf-8', 'ignore')
    except Exception:
        return None
    return {k: len(re.findall(p, txt)) for k, p in LOG_PATS.items()}


buf = {'truth': None, 'cmd': None, 'inf': None, 'occ': None, 'odom': None}


def odom_cb(msg):
    """规划器输入的世界系位姿 (与膨胀图同系) —— A* 起点的判据必须用它, 不能用真值。"""
    p = msg.pose.pose.position
    buf['odom'] = (p.x, p.y, p.z)


def truth_cb(msg):
    want = 'iris_lidar_%d' % ID
    for name, pose, tw in zip(msg.name, msg.pose, msg.twist):
        if name == want:
            buf['truth'] = (pose.position.x, pose.position.y, pose.position.z,
                            float(np.linalg.norm([tw.linear.x, tw.linear.y, tw.linear.z])))
            return


def cmd_cb(msg):
    buf['cmd'] = msg


def mk(key):
    def cb(msg):
        acc = [(p[0], p[1], p[2]) for p in
               pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)]
        buf[key] = np.array(acc) if acc else np.zeros((0, 3))
    return cb


rospy.init_node('stuck_probe', anonymous=True)
rospy.Subscriber('/gazebo/model_states', ModelStates, truth_cb)
rospy.Subscriber('/drone_%d_planning/pos_cmd' % ID, PositionCommand, cmd_cb)
rospy.Subscriber('/drone_%d_ego_planner_node/grid_map/occupancy_inflate' % ID,
                 PointCloud2, mk('inf'))
rospy.Subscriber('/drone_%d_ego_planner_node/grid_map/occupancy' % ID,
                 PointCloud2, mk('occ'))
rospy.Subscriber('/drone_%d_visual_slam/odom' % ID, Odometry, odom_cb)

print('drone_%d: 卡住判据 = v_real < %.2f m/s 持续 >= %.1f s; 采样 %.0f s'
      % (ID, V_TH, STUCK_T, DUR))

rows = []
offs = []                       # odom - truth, 用来核对"坐标口径"
c_log0 = log_counts()           # 采样前的日志事件计数
t0 = time.time()                # 用**墙钟**计时: gzserver 挂掉后 /clock 会冻结,
t_end = t0 + DUR                # 用 rospy.Time 会让循环永不结束 (2026-10-14 实测踩过)
while not rospy.is_shutdown() and time.time() < t_end:
    try:
        rospy.sleep(0.1)
    except rospy.exceptions.ROSInterruptException:
        print('\n[!] ROS 中断 (仿真正在退出 / master 掉线) —— 用已采到的数据继续出报告')
        break
    if buf['truth'] is None:
        continue
    tx, ty, tz, v_real = buf['truth']
    p_real = np.array([tx, ty, tz])
    cmd = buf['cmd']
    if cmd is None:
        tid, tflag, err = -1, -1, float('nan')
    else:
        tid = int(cmd.trajectory_id)
        tflag = int(cmd.trajectory_flag)
        err = float(np.linalg.norm(
            [cmd.position.x - tx, cmd.position.y - ty, cmd.position.z - tz]))
    inf, occ = buf['inf'], buf['occ']
    # d_inf 必须用**规划器同系**的位姿(odom)算: A* 起点 = odom 投影到栅格;
    # Gazebo 真值是 model origin, 比 odom(雷达点)低约 mount_z(默认 0.20 m),
    # 用真值算 d_inf 会系统性偏大 -> 漏判"起点非法"。
    p_geo = np.array(buf['odom']) if buf['odom'] is not None else p_real
    offs.append(p_geo - p_real)
    d_inf = float(np.min(np.linalg.norm(inf - p_geo, axis=1))) \
        if inf is not None and len(inf) else float('inf')
    d_obs = float(np.min(np.linalg.norm(occ - p_real, axis=1))) \
        if occ is not None and len(occ) else float('inf')
    rows.append((time.time() - t0, v_real, d_inf, d_obs, err,
                 tx, ty, tz, tid, tflag))

if not rows:
    sys.exit('没采到数据: 仿真在跑吗? %d 号机的 model_states 有吗?' % ID)

R = np.array(rows, dtype=float)
t, v_real = R[:, 0], R[:, 1]
print('采到 %d 帧, v_real 平均 %.3f m/s, 移动帧(v>0.1) = %d'
      % (len(R), v_real.mean(), int((v_real > 0.1).sum())))

if offs:
    A = np.array(offs)
    print('坐标口径核对: odom - truth = (%+.3f, %+.3f, %+.3f) m, 各轴 std=(%.3f, %.3f, %.3f)'
          % (A[:, 0].mean(), A[:, 1].mean(), A[:, 2].mean(),
             A[:, 0].std(), A[:, 1].std(), A[:, 2].std()))
    if abs(A[:, 2].mean()) > 0.1 and A[:, 2].std() < 0.02:
        print('  => z 上是**常数**偏移 (mount_z 安装高度, odom=雷达点/真值=模型原点), '
              '不是漂移; xy 偏移 <= %.3f m 说明水平对齐正常。' % np.hypot(A[:, 0].mean(), A[:, 1].mean()))
    print('  => 本脚本的 d_inf/在格内判定已改用 odom (与膨胀图同系, 也是 A* 起点的口径)。')

c_log1 = log_counts()
print('\nFSM 活动 (%.0f s 内 %s 尾部窗口新增):' % (DUR, os.path.basename(LOG_PATH)))
if c_log1 is None or c_log0 is None:
    print('  读不到日志(可用 PLANNER_LOG=/path/to/formation.log 指定) —— '
          '可改看 planner 终端里的 Emergency stop / Try to replan')
else:
    d = {k: max(0, c_log1[k] - c_log0.get(k, 0)) for k in c_log1}
    print('  replan 日志 %d 条 | "Try to replan" %d | "Emergency stop" %d | '
          '"current traj in collision" %d | "Depth Lost" %d | "Close to goal" %d'
          % (d['replan'], d['try_replan'], d['emergency'], d['collision'],
             d['depth_lost'], d['near_goal']))
    rate = d['replan'] / max(DUR, 1e-6)
    if d['replan'] > 5:
        print('  -> FSM 在**拼命重规划** (%.1f 次/s), 而上面 traj_id 没变:')
        print('     不是 FSM 停摆, 是每次 A*/reboundReplan 都**失败** -> '
              '对应"起点/终点非法"或 IMPOSSIBLE。')
    elif d['replan'] == 0:
        print('  -> FSM 一次重规划都没发起: 更像 fail-safe 未重新武装 / 停在 WAIT_TARGET')
        print('     (enable_fail_safe_ 只在收到新目标时复位) -> 直接再发一个 2D Nav Goal 试。')
    if d['depth_lost']:
        print('  -> 出现 %d 次 "Depth Lost": enable_fail_safe_ 被置 false。' % d['depth_lost'])

# ---------- 找"卡住事件" (连续静止段) ----------
events, start = [], None
for i in range(len(R)):
    if v_real[i] < V_TH and start is None:
        start = i
    elif v_real[i] >= V_TH and start is not None:
        events.append((start, i - 1))
        start = None
if start is not None:
    events.append((start, len(R) - 1))
events = [(a, b) for a, b in events if t[b] - t[a] >= STUCK_T]

if not events:
    print('\n没有 >= %.1f s 的静止段 -> 本次没复现"卡住不动"。' % STUCK_T)
    print('提示: 要复现请让轨迹贴着柱子飞 (或在 RViz 把目标点设在柱心附近), '
          '并把采样时长覆盖到卡住发生之后。')
    sys.exit(0)

print('\n==== 卡住事件 %d 段 ====' % len(events))
tot, inside_all, flag_all, tid_stuck = 0.0, 0, set(), 0
for a, b in events:
    ev = R[a:b + 1]
    dur = float(t[b] - t[a])
    tot += dur
    d_inf_ev = ev[:, 2]
    air = ev[:, 7] >= 0.2                        # 离地才判"被困"; 地面层是地图正常内容
    in_inflate = (d_inf_ev < HALF_VOX) & air
    inside_all += int(in_inflate.sum())

    tids = np.unique(ev[:, 8])
    flags = sorted(set(int(x) for x in np.unique(ev[:, 9])))
    flag_all |= set(flags)
    tid_stuck += int(tids.size <= 1)
    print('\n--- t = %.1f ~ %.1f s (持续 %.1f s) ---' % (t[a], t[b], dur))
    z_med = float(np.percentile(ev[:, 7], 50))
    print('  位置      : [%.2f, %.2f, %.2f]  z中位数=%.2f%s'
          % (ev[-1, 5], ev[-1, 6], ev[-1, 7], z_med,
             '  <- 在地面/未起飞' if z_med < 0.2 else ''))
    print('  d_inf     : min=%.3f m (到膨胀障碍面); 空中落在膨胀格内 = %d / %d 帧'
          '  (地面帧 %d 个不计)' % (d_inf_ev.min(), int(in_inflate.sum()), len(ev),
                                    int((~air).sum())))
    print('  d_obs     : min=%.3f m (到原始障碍图 occupancy; 若恒为 inf 说明该话题'
          '没发布出点, 本工程实测如此, 以 d_inf 为准)' % ev[:, 3].min())

    e4 = ev[:, 4][~np.isnan(ev[:, 4])]
    if len(e4):
        print('  err_track : p50=%.3f max=%.3f m (指令位置 vs 真值; 大=指令有、飞不动)'
              % (np.percentile(e4, 50), e4.max()))
    else:
        print('  err_track : 无数据 (该段没收到 pos_cmd -> 还没给目标 / 规划器没出指令)')

    print('  traj_id   : %s  (%s)'
          % (list(tids.astype(int)),
             '**事件期间没有任何新轨迹产生**' if tids.size <= 1 else '有新轨迹产生过'))
    print('  traj_flag : %s'
          % ', '.join('%d=%s' % (f, FLAG.get(f, '?')) for f in flags))

print('\n==== 汇总 ====')
print('卡住总时长 %.1f s / 采样 %.1f s; 落在膨胀障碍格内共 %d 帧' % (tot, DUR, inside_all))
print('出现过的 trajectory_flag: %s'
      % ', '.join('%d=%s' % (f, FLAG.get(f, '?')) for f in sorted(flag_all)))
print('其中 %d/%d 段"期间没有产生任何新轨迹"' % (tid_stuck, len(events)))
print('\n判读:')
if inside_all == 0 and float(np.median(R[:, 7])) < 0.2:
    print('  -> 所有静止事件的 z 中位数 %.2f m < 0.2: 机体在地面/未起飞 ——'
          % float(np.median(R[:, 7])))
    print('     这不是"规划卡住", 而是还没起飞 (或已落地)。起飞到 0.5 m 后再复现。')
elif inside_all:

    print('  -> 机体中心有 %d 帧落在**膨胀障碍格内**: A* 起点非法, planFromLocalTraj 必失败'
          % inside_all)
    print('     => "撞后卡住"的机械原因是"起点被困在膨胀体里", 不是规划器不想绕。')
    print('     对策: 1) 收到新目标会重新武装 fail-safe (源码已支持, 直接再发一个 goal);')
    print('           2) 撞后先把机体挪出膨胀体再给目标;')
    print('           3) 调小 obstacles_inflation (0.35->0.3) 缩小被困体积。')
elif 5 in flag_all:
    print('  -> 出现 ILLEGAL_START(5): traj_server 认为当前位姿偏离轨迹起点, 属同类问题。')
else:
    print('  -> 没看到"落在膨胀格内"/ILLEGAL_START: 静止更可能是 EMERGENCY_STOP 之后')
    print('     没收到新目标 (enable_fail_safe_ 未重新武装), 或 FSM 停在 WAIT_TARGET。')
    print('     请对照 planner 终端里有没有 "Emergency stop!" / "Depth Lost!" /')
    print('     "Try to replan a safe trajectory" / "Plan success when detect collision."')
    print('     再给一个 2D Nav Goal: 若收到目标就恢复 -> 就是 fail-safe 未武装那一条。')

