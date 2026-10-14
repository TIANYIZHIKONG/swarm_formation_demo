#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实测 "贴近障碍后局部地图把障碍擦掉" —— too_close 的感知侧根因验证。

三段链路 (2026-10-13 第三轮定位, 见 grid_map.cpp:raycastProcess 的注释):
  1) Gazebo 3D LiDAR <range><min> = 0.3 m (不能更小: 桨尖距雷达 0.18 m /
     机身顶面 0.20 m, 再小就把自己的桨扫进点云) -> 机体中心距柱面 < ~0.35 m
     时**该方向没有点云**;
  2) pcl_render_node 渲染出的 depth 图对应像素 = 0
     -> grid_map.projectDepthImage() 把 depth==0 当作"最远"
        (depth = max_ray_length + 0.1) 而不是"未知";
  3) raycastProcess() 这批 4.5 m 长射线把柱子的占据格按 -logit(p_miss) = -0.62
     每次往下削, 从 logit(0.8)=+1.39 掉到 logit(0.12)=-1.99 只要 6 次
     -> 柱子在地图上变成"自由" -> A*/优化器认为该处可穿过 -> 擦碰。

本脚本同时盯"真值几何"和"规划器地图", 直接判定第 2/3 段是否发生:
  d_true : 机体中心(真值) -> 最近柱面 的距离   [Gazebo 真值 + world 几何]
  d_map  : 机体中心 -> 规划器地图里最近障碍格 的距离 [grid_map/occupancy]
  n_near : 地图里距机体中心 < 1.0 m 的障碍格数
健康: d_true 变小时 d_map 跟着变小, n_near > 0 (障碍还在图上)
丢障: d_true < 0.5 m 而 d_map > d_true + 0.3 (或 n_near == 0) -> 就是第 3 段

用法 (先 source $TECH/sim_assets/env.sh; 仿真在跑; 在 RViz 里给一个会从柱群旁
擦过去的 2D Nav Goal, 让它有"贴近"的机会):
  python3 blind_probe.py 30 0        # 采样 30 s, drone_0
"""
import math
import re
import sys

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2

S = '/home/safina/keyan_project/something1017/TECH'
WORLD = S + '/sim_assets/worlds/formation.world'
MOUNT_X = 0.05          # 雷达相对机体中心的前置偏移 (gen_drone_sdf.py)
LIDAR_MIN = 0.3         # gen_drone_sdf.py 的 <range><min> = 物理盲区
BODY_R = 0.348          # 机体含桨尖半径
ZLO, ZHI = 0.15, 1.9    # 只看这个高度层的障碍 (柱身, 排除地面/柱顶)
NEAR = 1.0              # "近处障碍" 半径

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
ID = int(sys.argv[2]) if len(sys.argv) > 2 else 0

# ---------- world 真值: 柱心 / 半径 / 高度 ----------
pil = []
_txt = open(WORLD).read()
for m in re.finditer(r'<model name="(pillar_[^"]+)">(.*?)</model>', _txt, re.S):
    name, body = m.group(1), m.group(2)
    pm = re.search(r'<pose>\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', body)
    rm = re.search(r'<cylinder><radius>([\d.]+)</radius><length>([\d.]+)</length>', body)
    if not (pm and rm):
        continue
    x, y, z = float(pm.group(1)), float(pm.group(2)), float(pm.group(3))
    r, h = float(rm.group(1)), float(rm.group(2))
    if z + h / 2.0 < ZLO or z - h / 2.0 > ZHI:      # 不在飞行高度带 -> 不构成威胁
        continue
    pil.append((x, y, r))
CYL = np.array([(p[0], p[1]) for p in pil]) if pil else np.zeros((0, 2))
CYL_R = np.array([p[2] for p in pil]) if pil else np.zeros(0)
print('world 柱体(高度带内) %d 根' % len(pil))

buf = {'truth': None, 'occ': None}


def truth_cb(msg):
    want = 'iris_lidar_%d' % ID
    for name, pose in zip(msg.name, msg.pose):
        if name == want:
            p = pose.position
            buf['truth'] = (p.x, p.y, p.z)
            return


def cloud_cb(msg):
    acc = []
    for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
        if ZLO <= p[2] <= ZHI:
            acc.append((p[0], p[1], p[2]))
    buf['occ'] = np.array(acc) if acc else np.zeros((0, 3))


rospy.init_node('blind_probe', anonymous=True)
rospy.Subscriber('/gazebo/model_states', ModelStates, truth_cb)
rospy.Subscriber('/drone_%d_ego_planner_node/grid_map/occupancy' % ID, PointCloud2, cloud_cb)

print('采样 %.0f s (drone_%d) ...  说明: 让无人机从柱群旁边擦过去才有意义' % (DUR, ID))
rows = []
t_end = rospy.Time.now().to_sec() + DUR
while not rospy.is_shutdown() and rospy.Time.now().to_sec() < t_end:
    rospy.sleep(0.1)
    if buf['truth'] is None or buf['occ'] is None:
        continue
    tx, ty, _tz = buf['truth']
    if len(CYL):
        d_true = float(np.min(np.hypot(CYL[:, 0] - tx, CYL[:, 1] - ty) - CYL_R))
    else:
        d_true = float('nan')
    occ = buf['occ'][:, :2]
    if len(occ):
        dd = np.hypot(occ[:, 0] - tx, occ[:, 1] - ty)
        d_map = float(dd.min())
        n_near = int((dd < NEAR).sum())
    else:
        d_map, n_near = float('inf'), 0
    rows.append((rospy.Time.now().to_sec(), d_true, d_map, n_near))

if not rows:
    sys.exit('没有采到数据: 检查仿真是否在跑 / 该机是否有 grid_map/occupancy')

rows = np.array(rows, dtype=float)
i_min = int(np.argmin(rows[:, 1]))
print('\n==== 汇总 (drone_%d, %d 帧) ====' % (ID, len(rows)))
print('最近接近柱面  d_true_min = %.3f m  (第 %.1f s)' % (rows[i_min, 1], rows[i_min, 0] - rows[0, 0]))
print('该时刻地图最近障碍 d_map  = %.3f m' % rows[i_min, 2])
print('机体半径 %.3f m, 雷达盲区 %.2f m + 前置 %.2f m = %.2f m -> 地图"必然看得见"的下限'
      % (BODY_R, LIDAR_MIN, MOUNT_X, LIDAR_MIN + MOUNT_X))

close = rows[rows[:, 1] < 0.6]
if len(close) == 0:
    print('\n全程没有接近到 0.6 m 以内 -> 这次飞行无法判定, 请让轨迹贴着柱子飞')
else:
    lost = close[(close[:, 3] == 0) | (close[:, 2] > close[:, 1] + 0.3)]
    print('\n贴近帧 (d_true < 0.6 m): %d 帧' % len(close))
    print('  其中"地图丢障"帧    : %d 帧  <- 这个数必须是 0' % len(lost))
    if len(lost):
        print('  最早的 5 个丢障时刻 (t相对, d_true, d_map, n_near):')
        for r in lost[:5]:
            print('    t=%6.1f s  d_true=%.3f  d_map=%.3f  n_near=%d'
                  % (r[0] - rows[0, 0], r[1], r[2], r[3]))
        print('  -> 复现了感知盲区擦障: 检查 grid_map.cpp 的 min_ray_length 近距保护是否生效'
              ' (需要重编译 plan_env), 以及 grid_map/depth_filter_mindist 是否已降到 0.05')
    else:
        print('  地图始终保有近处障碍 -> 修复有效(或本次没贴到盲区内)')
print('\n提示: d_map 明显大于 d_true 只说明"地图比真值更保守/更远", 不是丢障;'
      ' 丢障的判据是 d_true 很小而 d_map 反而变大或近处格数归零。')
