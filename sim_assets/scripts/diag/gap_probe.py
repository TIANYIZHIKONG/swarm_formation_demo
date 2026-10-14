#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实测: 从规划器自己的 grid_map 量出"柱间可通行宽度"和"瓶颈", 判定算法能不能过、余量多少.

原理: occupancy_inflate 就是 A*/优化器真正用的硬障碍; 把它的每个连通域当成一根柱,
      两两求最近距离 = 地图允许机体穿越的最大宽度.
      再和 3 个门槛比:
        机体(含桨尖)直径 2*0.348 = 0.696 m -> 小于它则"地图说能过、机体过不去"
        ESDF 硬判据(A*第一遍) 额外 2*0.2 = 0.4 m
        优化器理想(obstacle_clearance=0.3, 第三轮改) 额外 2*0.3 = 0.6 m

用法 (先 source sim_assets/env.sh, 仿真在跑):
  python3 /tmp/gap_probe.py 10 0      # 采集 10 s, drone_0
"""
import sys

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from scipy import ndimage
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
ID = int(sys.argv[2]) if len(sys.argv) > 2 else 0
RES = 0.1
ZLO, ZHI = 0.25, 1.0        # 只取飞行高度层的点(柱身), 排除地面/柱顶
BODY_D = 2 * 0.348          # 机体含桨尖直径
ESDF_BAND = 0.4             # A* 第一遍的 ESDF 硬判定带 (2 x 0.2)
OPT_BAND = 0.6              # 优化器理想间隙 (2 x obstacle_clearance=0.3)

buf = {'raw': None, 'inf': None}


def mk(key):
    def cb(m):
        acc = []
        for p in pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True):
            if ZLO <= p[2] <= ZHI:
                acc.append((p[0], p[1]))
        if acc:
            buf[key] = np.array(acc)
    return cb


rospy.init_node('gap_probe', anonymous=True)
rospy.Subscriber('/drone_%d_ego_planner_node/grid_map/occupancy' % ID, PointCloud2, mk('raw'))
rospy.Subscriber('/drone_%d_ego_planner_node/grid_map/occupancy_inflate' % ID, PointCloud2, mk('inf'))
print('采集 %.0f s (drone_%d) ...' % (DUR, ID))
rospy.sleep(DUR)


def clusters(xy):
    """把点云栅格化 -> 连通域, 返回 [每个连通域的点坐标数组]"""
    if xy is None or len(xy) == 0:
        return [], None
    mins = xy.min(axis=0) - 3 * RES
    idx = np.floor((xy - mins) / RES).astype(int)
    shape = idx.max(axis=0) + 3
    occ = np.zeros(shape, dtype=bool)
    occ[idx[:, 0], idx[:, 1]] = True
    occ = ndimage.binary_closing(occ, np.ones((3, 3)))
    lab, n = ndimage.label(occ, np.ones((3, 3)))
    out = []
    nz = np.argwhere(occ)
    ls = lab[nz[:, 0], nz[:, 1]]
    for l in range(1, n + 1):
        pts = nz[ls == l]
        if len(pts) < 3:        # 噪声
            continue
        out.append(pts * RES + mins)
    return out, occ


def min_pair_dist(cl):
    """每对连通域之间的最小距离 -> 排序后的 (dist, i, j) 列表"""
    res = []
    for i in range(len(cl)):
        for j in range(i + 1, len(cl)):
            a, b = cl[i], cl[j]
            # 先用中心距粗筛 (可通行缝一定 < 3 m)
            if np.linalg.norm(a.mean(axis=0) - b.mean(axis=0)) > 3.5:
                continue
            d = cKDTree(b).query(a, k=1)[0].min()
            res.append((float(d), i, j))
    res.sort()
    return res


for key, tag, base in (('raw', '原始占用云(柱面)', 0.0), ('inf', '膨胀占用云(inflation=0.35)', 0.35)):
    cl, occ = clusters(buf[key])
    if not cl:
        print('%s: 没有数据(仿真没跑 / 该机没点云)' % tag)
        continue
    prs = min_pair_dist(cl)
    print('\n===== %s: %d 个障碍簇 =====' % (tag, len(cl)))
    if not prs:
        print('  相邻障碍间距都 > 3.5 m, 空间开阔')
        continue
    print('  最窄的 10 处两柱间隙 (面到面):')
    for d, i, j in prs[:10]:
        v = ''
        if base > 0:      # 膨胀图: 直接判机体能不能过
            v = '机体过不去(需%.3f)' % BODY_D if d < BODY_D else '机体可过(余量%.2f m/边)' % ((d - BODY_D) / 2)
        print('    gap = %.3f m   (簇#%d <-> 簇#%d)   %s' % (d, i, j, v))
    print('  统计: gap<%.3f(机体直径) 有 %d 处; gap<%.3f(+ESDF) 有 %d 处; gap<%.3f(优化器理想) 有 %d 处'
          % (BODY_D, sum(1 for d, _, _ in prs if d < BODY_D),
             BODY_D + ESDF_BAND, sum(1 for d, _, _ in prs if d < BODY_D + ESDF_BAND),
             BODY_D + OPT_BAND, sum(1 for d, _, _ in prs if d < BODY_D + OPT_BAND)))

if buf['raw'] is not None and buf['inf'] is not None:
    a, _ = clusters(buf['raw'])
    b, _ = clusters(buf['inf'])
    if a and b:
        pa, pb = min_pair_dist(a), min_pair_dist(b)
        if pa and pb:
            print('\n===== 感知/膨胀对比 =====')
            print('  原始柱面最窄缝 = %.3f m, 膨胀后最窄缝 = %.3f m -> 硬膨胀吃掉 %.3f m'
                  % (pa[0][0], pb[0][0], pa[0][0] - pb[0][0]))
            print('  (硬膨胀理论吃掉 2*0.35 = 0.70 m; 多出来的部分是体素量化 + 点云厚度)')
