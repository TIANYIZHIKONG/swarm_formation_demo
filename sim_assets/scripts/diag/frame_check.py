#!/usr/bin/env python3
"""frame_check.py -- 三方坐标系一致性检查 (判定 err_loc/err_ctrl 是"固定错位"还是"漂移")

为什么要查:
  obstacles_inflation = 0.35 m 是**规划器坐标系**里的安全余量。
  若"规划器坐标系(visual_slam odom)" 或 "控制器坐标系(mavros local_position)"
  与真实世界(Gazebo)存在**静态错位 d**, 则:
      飞机实际位置 = 规划器以为的位置 + d
  贴柱飞行时 d 会直接抵掉 0.35 m 的膨胀余量 -> 机体中心落进膨胀格 -> A* 起点非法 -> 卡死。

用法: python3 frame_check.py [DRONE_ID=0] [DURATION=15]
输出 4 组逐轴 mean/std:
  A) odom(visual_slam)      - truth(Gazebo)   地图/里程计错位或漂移
  B) mavros(local_position) - truth           控制器坐标系错位
  C) mavros                 - odom            规划器 vs 控制器坐标系错位 (**最致命**)
  D) pos_cmd                - truth / - odom  跟踪误差拆解
判读:
  |mean|>0.10 m 且 各轴 std<0.05 m  => 固定错位(可修: 对齐初始位姿/原点) -> 会系统性吃余量
  各轴 std>0.05 m                   => 随机漂移/滞后(可修: 定位/LIO, 降速)
"""
import sys
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from quadrotor_msgs.msg import PositionCommand

ID = int(sys.argv[1]) if len(sys.argv) > 1 else 0
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

buf = {}


def cb_truth(msg):
    for n, pose in zip(msg.name, msg.pose):
        if n == 'iris_lidar_%d' % ID:
            buf['truth'] = (pose.position.x, pose.position.y, pose.position.z)
            return


def cb_odom(msg):
    buf['odom'] = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                   msg.pose.pose.position.z)


def cb_mav(msg):
    buf['mav'] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)


def cb_cmd(msg):
    buf['cmd'] = msg


T = {'truth': ('/gazebo/model_states', ModelStates, cb_truth),
     'odom': ('/drone_%d_visual_slam/odom' % ID, Odometry, cb_odom),
     'mav': ('/drone_%d/mavros/local_position/pose' % ID, PoseStamped, cb_mav),
     'cmd': ('/drone_%d_planning/pos_cmd' % ID, PositionCommand, cb_cmd)}

rospy.init_node('frame_check', anonymous=True)
for k, (topic, typ, cb) in T.items():
    rospy.Subscriber(topic, typ, cb)

print('drone_%d: 采 %.0f s, 比对 真值(Gazebo) / visual_slam odom / mavros local / pos_cmd'
      % (ID, DUR))
print('  话题: %s' % ', '.join(T[k][0] for k in ('truth', 'odom', 'mav', 'cmd')))

rows = {'t': []}
t0 = rospy.Time.now().to_sec()
while not rospy.is_shutdown() and rospy.Time.now().to_sec() < t0 + DUR:
    rospy.sleep(0.05)
    if 'truth' not in buf or 'odom' not in buf:
        continue
    tr = np.array(buf['truth'])
    od = np.array(buf['odom'])
    rec = {'ot': od - tr}
    if 'mav' in buf:
        mv = np.array(buf['mav'])
        rec['mt'] = mv - tr
        rec['mo'] = mv - od
    if 'cmd' in buf:
        cm = np.array([buf['cmd'].position.x, buf['cmd'].position.y,
                       buf['cmd'].position.z])
        rec['ct'] = cm - tr
        rec['co'] = cm - od
    rows['t'].append(rospy.Time.now().to_sec() - t0)
    for k, v in rec.items():
        rows.setdefault(k, []).append(v)

if 'ot' not in rows or len(rows['ot']) < 5:
    sys.exit('没采到数据: 仿真在跑吗? %d 号机的 model_states / visual_slam odom 有吗?' % ID)


def stat(label, key, unit_note=''):
    if key not in rows:
        print('  %-22s (无数据: 该话题没收到)' % label)
        return
    A = np.array(rows[key], dtype=float)
    m, s = A.mean(axis=0), A.std(axis=0)
    n = np.linalg.norm(A, axis=1)
    print('  %-22s mean=(%+.3f,%+.3f,%+.3f) |mean|=%.3f | std=(%.3f,%.3f,%.3f) | '
          '|d| min/med/max=%.3f/%.3f/%.3f %s'
          % (label, m[0], m[1], m[2], np.linalg.norm(m), s[0], s[1], s[2],
             n.min(), np.median(n), n.max(), unit_note))
    return np.linalg.norm(m), s.max()


print('\n==== 逐轴偏差 (单位 m, 采样 %d 帧) ====' % len(rows['ot']))
r_mo = stat('mavros - odom  [C]', 'mo')
r_ot = stat('odom   - truth [A]', 'ot')
r_mt = stat('mavros - truth [B]', 'mt')
r_ct = stat('pos_cmd - truth[D1]', 'ct')
r_co = stat('pos_cmd - odom [D2]', 'co')

print('\n==== 判读 ====')
print('  (固定错位判据: |mean|>0.10 m 且 各轴 std<0.05 m; 漂移判据: 某轴 std>0.05 m)')
for label, r in (('C 规划器-控制器坐标系', r_mo), ('A 里程计-真值', r_ot),
                 ('B 控制器-真值', r_mt), ('D1 指令-真值', r_ct)):
    if r is None:
        continue
    mag, sd = r
    if mag > 0.10 and sd < 0.05:
        print('  %-20s -> **固定错位** %.3f m (std %.3f): 系统性吃余量' % (label, mag, sd))
    elif sd > 0.05:
        print('  %-20s -> 随机漂移/滞后 (std %.3f m)' % (label, sd))
    else:
        print('  %-20s -> 良好 (%.3f m, std %.3f)' % (label, mag, sd))

if r_mo is not None and r_mo[0] > 0.10 and r_mo[1] < 0.05:
    print('\n  !! [C] 规划器与控制器坐标系固定错位 -> 规划出的安全余量在实际飞行中被整体平移掉:')
    print('     贴柱时 |d|=%.2f m 直接抵掉 obstacles_inflation=0.35 m 的余量,' % r_mo[0])
    print('     机体中心易落进膨胀格 -> A* 起点非法 -> planFromLocalTraj 必失败 -> 撞后卡死。')
    print('     对策: 对齐 mavros local 与 visual_slam 的原点/初始位姿'
          ' (查 vio_offset.py / PX4 EKF origin / gazebo 初始 pose)。')
if r_ot is not None and r_ot[0] > 0.10 and r_ot[1] < 0.05:
    A = np.array(rows['ot'])
    xy = float(np.linalg.norm(A[:, :2].mean(axis=0)))
    print('\n  !! [A] 里程计相对真值固定错位 %.3f m (xy 分量 %.3f m, z 分量 %+.3f m):'
          % (r_ot[0], xy, A[:, 2].mean()))
    if xy < 0.05 < abs(A[:, 2].mean()):
        print('     几乎只在 z 上 -> 疑为 Gazebo model origin 与 base_link/CG 的高度定义差,')
        print('     会污染一切"用真值算 d_inf"的判据 (xy 判定仍可信, z 需扣掉该常数)。')
    else:
        print('     xy 也有错位 -> 地图/定位坐标系未与真值对齐, 直接吃膨胀余量。')
print('==== done ====')
