#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实测 "地图里的柱子只有 X m 高, 真值却有 Y m" —— 规划器因此从柱顶上方绕过。

背景 (几何复现):
  · world 里柱子都是从地面起的实心柱: z 0 ~ 2.50..3.46 m (平均 2.99, 40 根 >= 3.0)
  · 感知有两个**硬上限**, 决定了地图里柱子最多能建多高:
      1) LiDAR 垂直 FOV 只有 ±15° (gen_drone_sdf.py 的 vertical min/max_angle)
         -> 雷达所在高度能看到的上限 = z_lidar + d*tan(15°) = z_lidar + 0.268*d
      2) grid_map/max_ray_length = 4.5 m: 更远的点**只用来清障、不记为障碍**
         (grid_map.cpp:402-410 -> setCacheOccupancy(pt_w, 0) 即 miss)
         -> 有效观测半径 4.5 m
  => 地图里柱子最高点 ≈ (z_机 + 0.20) + 4.5*tan(15°) = z_机 + 1.41 m
     机体 z=0.5 时 ≈ 1.9 m; z=1.0 时 ≈ 2.4 m; 而柱顶 2.5~3.46 m
     -> 柱子上方在地图上是"假空白", A*/B 样条会从上方绕过 -> 撞柱顶。
  · grid_map/virtual_ceil_height 现在是 4.5 m, 完全压不住(它才是能真正把飞行
     高度带压到"地图可靠建高区"以内的那个开关)。

用法 (先 source $TECH/sim_assets/env.sh, 仿真在跑):
  python3 height_probe.py 5 0      # 采样 5 s, drone_0
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
MOUNT_Z = 0.20                     # 雷达相对机体中心的安装高度
LIDAR_VFOV = math.radians(15.0)    # gen_drone_sdf.py: vertical ±15°
MAX_RAY = 4.5                      # grid_map/max_ray_length

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
ID = int(sys.argv[2]) if len(sys.argv) > 2 else 0

# ---------- world 真值 ----------
pil = []
for m in re.finditer(r'<model name="(pillar_[^"]+)">(.*?)</model>', open(WORLD).read(), re.S):
    body = m.group(2)
    pm = re.search(r'<pose>\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', body)
    rm = re.search(r'<cylinder><radius>([\d.]+)</radius><length>([\d.]+)</length>', body)
    if not (pm and rm):
        continue
    pil.append((m.group(1), float(pm.group(1)), float(pm.group(2)),
                float(rm.group(1)), float(pm.group(3)) + float(rm.group(2)) / 2.0))
CYL = np.array([(p[1], p[2], p[3], p[4]) for p in pil])
print('world 柱体 %d 根, 真值柱顶 %.2f ~ %.2f m (平均 %.2f)'
      % (len(pil), min(p[4] for p in pil), max(p[4] for p in pil),
         np.mean([p[4] for p in pil])))

buf = {'truth': None, 'occ': None, 'inf': None}


def truth_cb(msg):
    want = 'iris_lidar_%d' % ID
    for name, pose in zip(msg.name, msg.pose):
        if name == want:
            buf['truth'] = (pose.position.x, pose.position.y, pose.position.z)
            return


def mk(key):
    def cb(msg):
        acc = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            if p[2] > -0.2:
                acc.append((p[0], p[1], p[2]))
        buf[key] = np.array(acc) if acc else np.zeros((0, 3))
    return cb


rospy.init_node('height_probe', anonymous=True)
rospy.Subscriber('/gazebo/model_states', ModelStates, truth_cb)
rospy.Subscriber('/drone_%d_ego_planner_node/grid_map/occupancy' % ID, PointCloud2, mk('occ'))
rospy.Subscriber('/drone_%d_ego_planner_node/grid_map/occupancy_inflate' % ID, PointCloud2, mk('inf'))

print('采样 %.0f s (drone_%d) ...' % (DUR, ID))
rospy.sleep(DUR)

if buf['occ'] is None or len(buf['occ']) == 0:
    sys.exit('没有 grid_map/occupancy 数据: 仿真没跑 / 该机还没建图')
tx, ty, tz = buf['truth'] if buf['truth'] else (0.0, 0.0, float('nan'))
occ = buf['occ']
print('\n机体真值 z = %.2f m, 雷达 z = %.2f m' % (tz, tz + MOUNT_Z))
print('理论: 地图可记为障碍的最高点 ≈ 雷达z + %.1f*tan(15°) = %.2f m'
      % (MAX_RAY, tz + MOUNT_Z + MAX_RAY * math.tan(LIDAR_VFOV)))

# ---------- 逐柱: 地图里的最高点 vs 真值柱顶 ----------
rows = []
for name, cx, cy, r, zhi in pil:
    d = math.hypot(cx - tx, cy - ty)
    if d > 6.0:                    # 只看机体附近(局部图)的柱子
        continue
    sel = occ[np.hypot(occ[:, 0] - cx, occ[:, 1] - cy) < r + 0.5]
    if len(sel) == 0:
        rows.append((name, d, r, zhi, float('nan'), float('nan')))
        continue
    zmap = float(sel[:, 2].max())
    rows.append((name, d, r, zhi, zmap, zhi - zmap))
rows.sort(key=lambda t: t[1])

print('\n==== 机体 6 m 内的柱子: 地图柱高 vs 真值柱顶 (%d 根) ====' % len(rows))
print('%-14s %6s %5s %8s %9s %7s  %s' % ('柱子', 'd/m', 'r/m', '真值zhi', '地图zmax', '缺口', '判定'))
nbad = 0
gaps = []
for name, d, r, zhi, zmap, gap in rows:
    if zmap != zmap:
        v, g = '地图无该柱(!)', float('nan')
    else:
        g = gap
        gaps.append(g)
        if gap > 0.3:
            v = '上方 %.2f m 假空 -> 可从上方绕' % gap
            nbad += 1
        elif gap > 0.0:
            v = '基本贴顶(差 %.2f)' % gap
        else:
            v = '地图比真值高(膨胀)'
    print('%-14s %6.2f %5.2f %8.2f %9s %7s  %s'
          % (name.replace('pillar_', ''), d, r, zhi,
             ('%.2f' % zmap) if zmap == zmap else 'n/a',
             ('%.2f' % g) if g == g else 'n/a', v))

if gaps:
    print('\n缺口(真值柱顶 - 地图最高点): 中位 %.2f m, 最大 %.2f m; 缺口 > 0.3 m 的柱子 %d/%d 根'
          % (float(np.median(gaps)), max(gaps), nbad, len(gaps)))

# ---------- 虚拟天花板检测 (它在 occupancy_inflate 里是一整层) ----------
inf = buf['inf']
if inf is not None and len(inf):
    zs = np.round(inf[:, 2] / 0.1) * 0.1
    vals, cnt = np.unique(zs, return_counts=True)
    k = int(np.argmax(cnt))
    print('\n膨胀图分层的最大点数: z=%.1f m 有 %d 个格子 (全部 %d 个格子)'
          % (vals[k], cnt[k], len(inf)))
    if cnt[k] > 500 and cnt[k] > 5 * np.median(cnt):
        print('  -> z=%.1f m 是一整层横铺的"虚拟天花板" (grid_map/virtual_ceil_height)'
              % vals[k])
    else:
        print('  -> 没有明显横铺的整层; 当前 virtual_ceil_height=4.5 m, 高出局部图范围,'
              ' 等于压不住飞行高度')

print('\n结论要点: 地图柱高明显低于真值柱顶时, 规划器会认为"柱子上方能飞".')
print('  修法一(参数, 推荐): 把 grid_map/virtual_ceil_height 从 4.5 压到 2.0 附近,')
print('    让飞行高度带上界 < 地图可靠建高(≈z_机+1.41 m) -> 柱子上方不再"假空";')
print('  修法二(场景): 把 world 柱高降到 <= 2.2 m, 让感知能完整覆盖;')
print('  修法三(感知): max_ray_length 4.5 -> 9.0 等, 代价是地图体积/算量大幅上升。')
