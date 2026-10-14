#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实测 "转弯需求 vs 运动学/平滑性能力" —— 判断"来不及从两侧拐过去"到底是不是
被曲率/加速度(平滑性)限制卡住。

机制背景 (本工程里"平滑性"作用在哪一步):
  · 轨迹**形状**(B 样条)由优化器权重平衡决定 (config/normal_hexagon.yaml):
      weight_obstacle 50000 / weight_swarm 50000 / weight_formation 15000
      / weight_feasibility 10000 / obstacle_clearance 0.3 / swarm_clearance 1.0;
  · "平滑/可行性" = **软代价**(weight_feasibility) + 一次**段时长重分配**
    (refineTrajAlgo: 段太短就拉长时间, 保证不超 max_vel/max_acc)。
    注意它只改**时间分配**, 不会把轨迹形状改直;
  · 真正限制"能拐多急"的是运动学:
      κ_max = a_lat_max / v² ,  R_min = v² / a_lat_max  (a_lat_max ≈ manager/max_acc)
    manager/feasibility_tolerance = 0.1 是"允许超限 10%"的比例, 属**放宽**不是收紧;
  · 另一条隐藏限制: FSM 的 checkCollisionCallback 只看轨迹的**前 2/3**
    (t < getCollisionCheckTimeEnd()), 后 1/3 出了碰撞不告警 -> 障碍"闪入"轨迹
    后段时反应会晚一拍。

三分法判据 (脚本直接算, 避免"感觉是被平滑卡住"):
  A. use = a_lat_cmd / max_acc 的 p95 > 0.85
     -> 轨迹确实贴着运动学极限: 只能提高 max_acc 或降低 max_vel (R = v²/a);
  B. use_p95 < 0.60 却仍撞上
     -> 不是"来不及拐弯": 形状是被别的软权重拉住 (weight_formation/swarm_clearance),
        或障碍是"闪入"的 (感知丢障 -> 见 blind_probe.py);
  C. 指令合理但实际位姿跟不上 (err_track 大)
     -> 控制器/定位滞后, 与规划器无关。

输出量:
  v_cmd/a_lat_cmd (轨迹指令)   v_real/a_lat_real (Gazebo 真值)
  err_track = |pos_cmd - pos_true|      d_inf = 到膨胀图最近障碍格的距离
  in_inflate = (d_inf < 0.06) 即机体中心落在膨胀障碍格内 -> A* 起点非法

用法 (先 source $TECH/sim_assets/env.sh; 仿真在跑):
  python3 corner_probe.py 30 0            # 采样 30 s, drone_0, 按默认 a_max=2.5
  python3 corner_probe.py 30 0 6.0 1.5    # 若命令行改过 max_acc/max_vel, 显式传入

本工程实测注意 (2026-10-13):
  · pos_cmd 的 position 在轨迹走完后会**停在终点**: 若 err_track 是近常数(脚本会算
    std), 那是"指令已在终点", 不是跟踪滞后;
  · 机体在地面时 (z<0.2 m) 膨胀图必然包住机体(地面层/起飞点被建图), 所以"落在膨胀
    格内"只在 z>=0.2 m 时统计 —— 先起飞到 0.5 m 再测;
  · occupancy 话题经常一帧点都不发(d_obs 恒 inf), 以膨胀图 d_inf 为准。
  · 【2026-10-14 实测修正】err_loc(odom vs 真值) 是 0.186 m 的**常数**(各轴 std≈0.001):
    那是 drone_all.launch 里 offset_z = mount_z = 0.20 的"雷达安装高度"定义差
    (odom 表示雷达点, Gazebo 真值是模型原点), **不是定位漂移**, 不要去修 LIO;
    所以 d_inf / "落在膨胀格内" 已改用 odom(与膨胀图同系) 判定, 而不是真值;
  · 运动段的 err_track 天然含"前馈领先"(cmd 是轨迹上的期望点, 超前真值 0.2~0.6 m 属正常);
    评判控制器/定位只看**静止帧** err_track 或 frame_check.py 的逐轴偏差;
  · 计时全部用**墙钟** time.time(): gzserver 挂掉后 /clock 会冻结, 用 rospy.Time 会挂死。


A/B 对照实验 (复现同一个"贴着柱子拐弯"的场景, 每次只改一个参数, 各跑一次本脚本):
  基线 : roslaunch sim_utils formation.launch
  实验1: ... max_acc:=5.0                      # 若判据 [A] 成立, use_p95 应上升且不再撞
  实验2: ... max_vel:=0.5                      # 降速直接降低所需 a_lat (∝ v²)
  实验3: 改 config/normal_hexagon.yaml: weight_formation 15000 -> 5000   # 检验 [B]
  实验4: 同上: obstacle_clearance 0.3 -> 0.4, swarm_clearance 1.0 -> 0.7
  每组都记录: use p95 / 最小 d_inf / 是否到达目标, 用列出的三条判据(A/B/C)对号入座。
"""

import sys
import time

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
ID = int(sys.argv[2]) if len(sys.argv) > 2 else 0
A_MAX = float(sys.argv[3]) if len(sys.argv) > 3 else 2.5     # manager/max_acc
V_MAX = float(sys.argv[4]) if len(sys.argv) > 4 else 0.8     # manager/max_vel
HALF_VOX = 0.06             # 0.1 m 体素的半格, 判"落在障碍格内"
DANGER = 0.5                # 只看膨胀面 0.5 m 以内的"危险帧"

buf = {'truth': None, 'cmd': None, 'inf': None, 'cmd_t': 0.0,
       'odom': None, 'odom_t': 0.0}


def odom_cb(msg):
    """规划器输入的世界系位姿(visual_slam/odom 或 mavros PoseStamped 都兼容)。"""
    p = msg.pose.pose.position if hasattr(msg.pose, 'pose') else msg.pose.position
    buf['odom'] = np.array([p.x, p.y, p.z])
    buf['odom_t'] = time.time()          # 墙钟: /clock 冻结时 rospy.Time 会停住




def truth_cb(msg):
    want = 'iris_lidar_%d' % ID
    for name, pose, tw in zip(msg.name, msg.pose, msg.twist):
        if name == want:
            buf['truth'] = (pose.position.x, pose.position.y, pose.position.z,
                            tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.z)
            return


def cmd_cb(msg):
    buf['cmd'] = msg
    buf['cmd_t'] = time.time()



def inf_cb(msg):
    acc = [(p[0], p[1], p[2]) for p in
           pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)]
    buf['inf'] = np.array(acc) if acc else np.zeros((0, 3))


rospy.init_node('corner_probe', anonymous=True)
try:
    _topics = [t for t, _ in rospy.get_published_topics()]
except Exception as _e:
    sys.exit('[!] 连不上 ROS master (%s) —— 先启动仿真: ./run_formation.sh 3' % _e)
if '/drone_%d_planning/pos_cmd' % ID not in _topics:
    print('[!] 警告: 没看到 /drone_%d_planning/pos_cmd (仿真可能还没就绪或崩了)' % ID)
rospy.Subscriber('/gazebo/model_states', ModelStates, truth_cb)
rospy.Subscriber('/drone_%d_planning/pos_cmd' % ID, PositionCommand, cmd_cb)
rospy.Subscriber('/drone_%d_ego_planner_node/grid_map/occupancy_inflate' % ID,
                 PointCloud2, inf_cb)
ODOM_TOPIC = rospy.get_param('~odom_topic', '/drone_%d_visual_slam/odom' % ID)
rospy.Subscriber(ODOM_TOPIC, Odometry, odom_cb)


print('drone_%d: a_max=%.2f m/s2, v_max=%.2f m/s -> R_min = %.3f m @v_max'
      % (ID, A_MAX, V_MAX, V_MAX ** 2 / A_MAX))
print('采样 %.0f s ...  (请给一个会贴着柱子过去的 2D Nav Goal)' % DUR)

rows, ages = [], []
t_wall = time.time()                 # 墙钟计时: gzserver 挂掉后 /clock 冻结,
t_end = t_wall + DUR                 # 用 rospy.Time 会让循环永不结束 (2026-10-14 实测踩过)

while not rospy.is_shutdown() and time.time() < t_end:
    try:
        rospy.sleep(0.1)
    except rospy.exceptions.ROSInterruptException:
        print('\n[!] ROS 中断 (仿真正在退出 / master 掉线) —— 用已采到的数据继续出报告')
        break
    if buf['truth'] is None or buf['cmd'] is None:
        continue
    tx, ty, tz, vx, vy, vz, wz = buf['truth']
    p_real = np.array([tx, ty, tz])
    v_real = float(np.hypot(vx, vy))
    a_lat_real = v_real * abs(wz)                       # a_lat = v * omega (水平)

    c = buf['cmd']                      # 用最新一帧指令, 不消费掉: 静止时 pos_cmd
    ages.append(time.time() - buf['cmd_t'])   # 可能很久才更新一次

    p_cmd = np.array([c.position.x, c.position.y, c.position.z])
    v_cmd_v = np.array([c.velocity.x, c.velocity.y, c.velocity.z])
    a_cmd_v = np.array([c.acceleration.x, c.acceleration.y, c.acceleration.z])
    v_cmd = float(np.linalg.norm(v_cmd_v))
    if v_cmd > 1e-3:
        vhat = v_cmd_v / v_cmd
        a_lat_cmd = float(np.linalg.norm(a_cmd_v - (a_cmd_v @ vhat) * vhat))
    else:
        a_lat_cmd = 0.0
    kappa = a_lat_cmd / (v_cmd ** 2 + 1e-9)
    err_track = float(np.linalg.norm(p_cmd - p_real))
    err_loc = (float(np.linalg.norm(buf['odom'] - p_real))
               if buf['odom'] is not None else float('nan'))       # 定位漂移
    err_ctrl = (float(np.linalg.norm(p_cmd - buf['odom']))
                if buf['odom'] is not None else float('nan'))      # 控制误差


    inf = buf['inf']
    # d_inf 用**规划器同系**的 odom 算 (A* 起点 = odom 投影到栅格): Gazebo 真值是模型原点,
    # 比 odom(雷达点)低约 mount_z=0.20 m, 用真值算会系统性偏大 -> 漏判"起点非法"。
    p_geo = buf['odom'] if buf['odom'] is not None else p_real
    d_inf = float(np.min(np.linalg.norm(inf - p_geo, axis=1))) \
        if inf is not None and len(inf) else float('inf')

    rows.append((time.time() - t_wall, v_cmd, a_lat_cmd, kappa,
                 v_real, a_lat_real, err_track, d_inf, float(c.trajectory_id), tz,
                 err_loc, err_ctrl))



if not rows:
    sys.exit('没采到数据: 仿真在跑吗? %d 号机的 pos_cmd / occupancy_inflate 有吗?' % ID)

R = np.array(rows, dtype=float)
use_cmd = R[:, 2] / A_MAX
use_real = R[:, 5] / A_MAX
moving = R[:, 4] > 0.1            # 真值在动
cmd_move = R[:, 1] > 0.1          # 指令在动: a_lat 的分位数必须以它为口径,
mset = cmd_move if cmd_move.any() else moving   # 否则被"指令已停"的帧稀释


print('\n==== 汇总 (drone_%d, %d 帧, %.0f s) ====' % (ID, len(R), DUR))
print('pos_cmd 帧龄: p50=%.2f p95=%.2f max=%.2f s  (很大 => 规划器很久没发出新指令)'
      % (np.percentile(ages, 50), np.percentile(ages, 95), max(ages)))

print('轨迹指令: v_cmd   p50=%.2f p95=%.2f max=%.2f m/s   (>v_max: %d 帧)'
      % (np.percentile(R[:, 1], 50), np.percentile(R[:, 1], 95), R[:, 1].max(),
         int((R[:, 1] > V_MAX + 1e-3).sum())))
print('          a_lat   p50=%.2f p95=%.2f max=%.2f m/s2'
      % (np.percentile(R[:, 2], 50), np.percentile(R[:, 2], 95), R[:, 2].max()))
if mset.any():
    print('  机动能力使用率 a_lat/a_max: 全帧 p95=%.2f | 指令运动帧(N=%d) p50=%.2f p95=%.2f max=%.2f'
          % (np.percentile(use_cmd, 95), int(cmd_move.sum()),
             np.percentile(use_cmd[mset], 50), np.percentile(use_cmd[mset], 95),
             use_cmd[mset].max()))
else:
    print('  机动能力使用率 a_lat/a_max: 全帧 p95=%.2f | **采样期内指令速度恒为 0**'
          ' (没有目标 / 轨迹已执行完) -> 机动能力无从判定' % np.percentile(use_cmd, 95))
    print('     请先在 RViz 给一个会让它贴着柱子飞过去的 2D Nav Goal, 再跑本脚本。')

k95 = float(np.percentile(R[:, 3], 95))
print('  指令曲率 kappa p95=%.3f 1/m -> 需要 R = %.2f m (运动学下限 R_min=%.3f m @v=%.2f)'
      % (k95, 1.0 / max(k95, 1e-9), V_MAX ** 2 / A_MAX, V_MAX))
print('真值飞行: v_real p50=%.2f p95=%.2f max=%.2f m/s'
      % (np.percentile(R[:, 4], 50), np.percentile(R[:, 4], 95), R[:, 4].max()))
print('          a_lat_real p50=%.2f p95=%.2f max=%.2f m/s2 (使用率 p95=%.2f)'
      % (np.percentile(R[:, 5], 50), np.percentile(R[:, 5], 95), R[:, 5].max(),
         np.percentile(use_real, 95)))
print('          z_real p50=%.2f min=%.2f max=%.2f m  (z<0.2 的帧 = %d: 在地面/未起飞)'
      % (np.percentile(R[:, 9], 50), R[:, 9].min(), R[:, 9].max(),
         int((R[:, 9] < 0.2).sum())))
err_std = float(np.std(R[:, 6]))
print('跟踪误差 err_track: p50=%.3f p95=%.3f max=%.3f m  std=%.3f  (std<0.05 => 近常数,'
      ' 多半是"指令停在终点/原点差"而非跟踪滞后)'
      % (np.percentile(R[:, 6], 50), np.percentile(R[:, 6], 95), R[:, 6].max(), err_std))
still = R[:, 4] < 0.1
if still.any():
    e_st = R[still, 6]
    print('  静止帧(v_real<0.1, N=%d) err_track: p50=%.3f max=%.3f m   <== 这个口径才排除'
          '"轨迹前馈领先"'
          % (int(still.sum()), np.percentile(e_st, 50), e_st.max()))
    print('     (飞行中 cmd 是轨迹上的**期望点**, 天然超前真值 0.2~0.6 m 属正常;'
          ' 静止时仍大才是坐标系/定位问题)')
    if float(np.percentile(e_st, 50)) > 0.2:
        print('     静止时仍 >0.2 m -> 是**坐标系/定位**问题 (逐轴核对用 frame_check.py),'
              ' 不是控制器滞后。')

loc = R[:, 10][~np.isnan(R[:, 10])]
ctl = R[:, 11][~np.isnan(R[:, 11])]
if len(loc):
    print('  └ 拆分: 定位/坐标 err_loc(%s vs 真值) p50=%.3f p95=%.3f max=%.3f mean=%.3f std=%.3f m'
          % (ODOM_TOPIC.replace('/drone_%d' % ID, ''), np.percentile(loc, 50),
             np.percentile(loc, 95), loc.max(), float(np.mean(loc)), float(np.std(loc))))
    if float(np.std(loc)) < 0.02 and abs(float(np.mean(loc)) - 0.20) < 0.08:
        print('     (近常数 %.2f m ≈ mount_z=0.20 -> 这是 odom=雷达点 / 真值=模型原点的'
              '**安装高度差**, 不是定位漂移, 别去修 LIO)' % float(np.mean(loc)))
    elif float(np.std(loc)) < 0.02:
        print('     (近常数 %.2f m 又不是 mount_z -> 坐标系静态错位, 会系统性吃掉膨胀余量)'
              % float(np.mean(loc)))
    else:
        print('     (有波动 std=%.3f -> 含真实漂移/滞后, 逐轴核对用 frame_check.py)'
              % float(np.std(loc)))
    print('     控制误差 err_ctrl(pos_cmd vs 定位)  p50=%.3f p95=%.3f max=%.3f m'
          % (np.percentile(ctl, 50), np.percentile(ctl, 95), ctl.max()))
    print('     (运动时 err_ctrl 偏大 = 轨迹前馈 + 机体滞后, 属正常量级;'
          ' 只有静止时仍大才是控制器问题)')
    print('     (只有 err_loc 有波动 std>0.02 才算定位漂移; 静止时 err_ctrl 大才是控制器问题)')
else:
    print('  └ 拆分: 没收到 %s -> 无法分离定位与控制 (可用 _odom_topic:=<话题> 指定)'
          % ODOM_TOPIC)



danger = R[R[:, 7] < DANGER]
i1 = int(np.argmin(R[:, 7]))
above = np.where(R[:i1 + 1, 7] >= 2.0)[0]     # 最近点之前, 最后一次离障碍还 >2 m
if len(above) and len(R) > 3:
    i0 = int(above[-1])
    dt = float(R[i1, 0] - R[i0, 0])
    seg = R[i0:i1 + 1]
    print('\n接近段 (最后一次 d>=2 m -> 最近点): t=%.1f s (d=%.2f) -> t=%.1f s (d=%.2f) 共 %.2f s'
          % (R[i0, 0], R[i0, 7], R[i1, 0], R[i1, 7], dt))
    print('  该段 a_max 允许的侧向位移 = 0.5*a_max*dt² = %.2f m, 实际指令 a_lat p95 = %.2f m/s²'
          % (0.5 * A_MAX * dt * dt, np.percentile(seg[:, 2], 95)))
    if dt > 10.0:
        print('  (dt>10 s: 机体是在障碍附近长时间徘徊, 不是"看见->逼近"一次过程,'
              ' 该位移上限失效, 请看下面的接近速率/危险区停留)')
    elif dt < 0.5 or not mset.any():
        print('  (窗口太短或机体没动 -> 无判定意义)')
    else:
        print('  (上限远大于绕行所需而仍没绕开 => 支持 [B] 不是"来不及")')

dd = np.diff(R[:, 7]) / np.maximum(np.diff(R[:, 0]), 1e-6)
if len(dd):
    k = int(np.argmin(dd))
    print('  最快接近速率 = %.2f m/s (t=%.1f s, 此时 d=%.2f m): 以该速率穿过 0.5 m 走廊只要 %.2f s'
          % (dd[k], R[k, 0], R[k, 7], 0.5 / max(1e-6, abs(dd[k]))))
n_danger = int((R[:, 7] < DANGER).sum())
print('  危险区(d<%.1f m)停留 = %d 帧 (约 %.1f s); 指令速度≈0 而真值速度>0.15 = %d 帧'
      ' (碰撞弹开/超调/漂移的证据)'
      % (DANGER, n_danger, 0.1 * n_danger,
         int(((R[:, 1] < 0.05) & (R[:, 4] > 0.15)).sum())))



print('\n危险帧 (到膨胀障碍面 < %.1f m): %d 帧' % (DANGER, len(danger)))

if len(danger):
    d_use = danger[:, 2] / A_MAX
    print('  这些帧: v_cmd p50=%.2f, a_lat/a_max p50=%.2f p95=%.2f max=%.2f'
          % (np.percentile(danger[:, 1], 50), np.percentile(d_use, 50),
             np.percentile(d_use, 95), d_use.max()))
    print('  最小膨胀面距离 = %.3f m; err_track>0.3 m 的帧 = %d'
          % (danger[:, 7].min(), int((danger[:, 6] > 0.3).sum())))
    d_air = danger[danger[:, 9] >= 0.2]
    if len(d_air):
        print('  其中空中(z>=0.2 m)落在膨胀障碍格内的帧 = %d  (A* 起点非法)'
              % int((d_air[:, 7] < HALF_VOX).sum()))
    else:
        print('  危险帧全在地面附近(z<0.2 m): 地图含地面层/起飞点, 不能据此判定"被困"')
else:
    print('  全程没进过 %.1f m -> 本次无法判定"来不及拐弯", 请让轨迹贴柱子过' % DANGER)

in_inf = R[(R[:, 7] < HALF_VOX) & (R[:, 9] >= 0.2)]
n_ground = int(((R[:, 7] < HALF_VOX) & (R[:, 9] < 0.2)).sum())
print('\n机体中心落在膨胀障碍格内: 空中 %d 帧(判定用); 地面附近(z<0.2)另有 %d 帧'
      ' (地图含地面层, 不能据此判"被困")' % (len(in_inf), n_ground))


print('\n==== 判据 ====')
if not mset.any():
    print('无人机在采样期内基本没动 (指令/真值速度 <0.1): 没有飞行数据可判定。')

    if float((R[:, 9] < 0.2).mean()) > 0.5:
        print('  且 z_real 中位数 = %.2f m (<0.2): 机体还在地面/未起飞 —— '
              '先起飞到 0.5 m 再测"来不及拐弯"。' % np.percentile(R[:, 9], 50))
    print('若它是"撞后卡住"了, 用 stuck_probe.py 看卡在哪一环。')

else:
    u95 = float(np.percentile(use_cmd[mset], 95))
    print('指令运动帧的 a_lat_cmd/a_max p95 = %.2f  (口径: pos_cmd 速度 >0.1 的帧)' % u95)

    if u95 > 0.85:
        print('  -> [A] 轨迹基本用满了 max_acc: **确实受运动学/平滑性限制**。')
        print('     对策: 提高 max_acc (R=v²/a 变小) 或降低 max_vel:')
        print('     roslaunch sim_utils formation.launch max_acc:=5.0 max_vel:=0.6')
    elif u95 < 0.60:
        print('  -> [B] 轨迹远没用满侧向机动能力: **不是"来不及拐弯"**。')
        print('     形状更可能被软权重拉住 (weight_formation 15000 / swarm_clearance 1.0),')
        print('     或障碍是"闪入"的 (先用 blind_probe.py 查有没有丢障)。')
    else:
        print('  -> 介于 A/B 之间: 轻微受限, 优先查是否"闪入" (blind_probe.py)。')
if len(in_inf):
    print('  -> 空中已有 %d 帧机体中心在膨胀障碍格内: A* 起点非法 -> planFromLocalTraj 必失败'
          ' -> 表现为"撞后卡住" (用 stuck_probe.py 细看)' % len(in_inf))
if float(np.percentile(R[:, 6], 95)) > 0.3 and err_std > 0.05:
    print('  -> 跟踪误差 p95 > 0.3 m 且 std > 0.05: 运动段大多含"轨迹前馈领先"(cmd 是期望点),'
          ' 只有**静止帧 err_track 仍大**才算 [C] 控制器/定位滞后 (见上面静止帧一行)。')
elif float(np.percentile(R[:, 6], 95)) > 0.3:
    print('  -> 跟踪误差大但 std 很小(近常数): 多半是"指令已停在终点/原点差",'
          ' 不是跟踪滞后, 不要误判为 [C]。')


