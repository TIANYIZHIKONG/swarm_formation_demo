#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线复现 A* 判据: 用 world 真值几何 + grid_map 的膨胀/ESDF 规则, 量化
   (1) 最窄的几处柱间缝能不能被"规划连通"; (2) 按最优路径走过去, 机体离柱面最近能有多少.
判据 (与代码一一对应):
  膨胀: inf_step = ceil(inflation/res), 球模板 (grid_map.cpp:634, inflatePoint)
  判据B(回退): occupancy_buffer_inflate == 1 即障碍 (dyn_a_star.h:63)
  判据A(第一遍): getDistance(pos) < 0.2 即障碍; getDistance 是"到膨胀面"的距离
                (dyn_a_star.h:65 + grid_map.cpp:1123 + grid_map.h:550 -> distance_buffer_all_)
  路径: widest-path(最大化路径上最小柱面距离), 代表 EGO 倾向于远离障碍的行为
"""
import heapq
import math
import re

import numpy as np
from scipy import ndimage

W = '/home/safina/keyan_project/something1017/TECH/sim_assets/worlds/formation.world'
RES = 0.1
X0, X1, Y0, Y1 = -20.0, 20.0, -15.0, 15.0      # map 40 x 30 m
BODY_R = 0.348                                  # 机体含桨尖半径
ESDF_TH = 0.2                                   # dyn_a_star.h:65

pil = []
for m in re.finditer(r'<model name="(pillar_[^"]+)">(.*?)</model>', open(W).read(), re.S):
    b = m.group(2)
    pm = re.search(r'<pose>\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', b)
    rm = re.search(r'<cylinder><radius>([\d.]+)</radius><length>([\d.]+)</length>', b)
    if pm and rm:
        pil.append((m.group(1), float(pm.group(1)), float(pm.group(2)),
                    float(rm.group(1)), float(rm.group(2))))

NX = int(round((X1 - X0) / RES))
NY = int(round((Y1 - Y0) / RES))
xs = X0 + (np.arange(NX) + 0.5) * RES
ys = Y0 + (np.arange(NY) + 0.5) * RES
XX, YY = np.meshgrid(xs, ys, indexing='ij')

occ = np.zeros((NX, NY), bool)                 # 真值柱面占用 (体素中心落在圆内)
for n, x, y, r, h in pil:
    occ |= (XX - x) ** 2 + (YY - y) ** 2 <= r * r

d_body = ndimage.distance_transform_edt(~occ) * RES     # 到最近柱面的距离, 单位 m
NB8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def disk(rc):
    y, x = np.ogrid[-rc:rc + 1, -rc:rc + 1]
    return x * x + y * y <= rc * rc


def to_idx(px, py, free):
    i = int((px - X0) / RES); j = int((py - Y0) / RES)
    i = min(max(i, 0), NX - 1); j = min(max(j, 0), NY - 1)
    if free[i, j]:
        return i, j
    _, ind = ndimage.distance_transform_edt(~free, return_indices=True)
    return int(ind[0, i, j]), int(ind[1, i, j])


def widest(free, si, sj):
    """从 (si,sj) 出发最大化"路径上最小 d_body"; 返回 best 数组 (-1 = 不可达)"""
    best = np.full(free.shape, -1.0)
    best[si, sj] = d_body[si, sj]
    pq = [(-best[si, sj], si, sj)]
    while pq:
        nb, i, j = heapq.heappop(pq)
        nb = -nb
        if nb < best[i, j] - 1e-12:
            continue
        for di, dj in NB8:
            ni, nj = i + di, j + dj
            if ni < 0 or nj < 0 or ni >= NX or nj >= NY or not free[ni, nj]:
                continue
            v = min(nb, d_body[ni, nj])
            if v > best[ni, nj]:
                best[ni, nj] = v
                heapq.heappush(pq, (-v, ni, nj))
    return best


# ---------------- 测试用例: 最窄的 6 处缝 + 3 个场角 ----------------
import itertools

pairs = []
for a, b in itertools.combinations(pil, 2):
    d = math.hypot(a[1] - b[1], a[2] - b[2])
    pairs.append((d - a[3] - b[3], a, b))
pairs.sort(key=lambda t: t[0])

targets = []
for gap, a, b in pairs[:6]:
    mx, my = (a[1] + b[1]) / 2, (a[2] + b[2]) / 2
    dx, dy = b[1] - a[1], b[2] - a[2]
    L = math.hypot(dx, dy)
    nx, ny = -dy / L, dx / L                       # 垂直于两柱连线
    targets.append(('%s<->%s' % (a[0][7:], b[0][7:]), gap, mx + 4 * nx, my + 4 * ny))
targets += [('map 右上(15,12)', -1, 15, 12), ('map 右下(15,-12)', -1, 15, -12),
            ('map 左上(-15,12)', -1, -15, 12)]

print('柱体 %d 根, 最窄柱面间隙 = %.3f m (%s <-> %s)' % (
    len(pil), pairs[0][0], pairs[0][1][0], pairs[0][2][0]))
print('机体含桨尖半径 = %.3f m -> 物理上要求柱面间隙 > %.3f m\n' % (BODY_R, 2 * BODY_R))

# 当前生效值 = 0.35 (advanced_param.xml / formation.launch 的 obstacles_inflation);
# 0.30 = 第三轮之前的旧值 (硬边界 0.316 m < 机体半径 0.348 m, 允许擦碰);
# 0.45 与 0.50 同为 ceil(4.5/5.0)=5 体素 -> 硬边界一样, 不要盲目加大。
for inf in (0.20, 0.30, 0.35, 0.45, 0.50):
    step = int(math.ceil(inf / RES - 1e-9))
    infl = ndimage.binary_dilation(occ, disk(step)) if step > 0 else occ.copy()
    esdf = ndimage.distance_transform_edt(~infl) * RES
    freeB = ~infl                    # 判据B: 膨胀判据 (A* 回退路径)
    freeA = esdf >= ESDF_TH          # 判据A: ESDF 0.2 判据 (A* 第一遍)
    siB, sjB = to_idx(0.0, 0.0, freeB)
    siA, sjA = to_idx(0.0, 0.0, freeA)
    bestB = widest(freeB, siB, sjB)
    bestA = widest(freeA, siA, sjA)
    dbB = d_body[freeB].min()        # 地图允许机体中心离柱面最近的距离
    dbA = d_body[freeA].min() if freeA.any() else float('nan')

    print('=' * 78)
    print('inflation=%.2f -> inf_step=%d 体素, 硬边界(可通行格内)离柱面最近 %.3f m'
          % (inf, step, dbB))
    if dbB < BODY_R:
        print('  [!] 地图认为可通行的位置, 机体已插入柱体 %.3f m -> 规划允许的擦碰' % (BODY_R - dbB))
    else:
        print('  [OK] 硬边界 >= 机体半径, 不擦碰')
    print('  ESDF(A)判据的可通行格: 离柱面最近 %.3f m (希望 >= %.3f)' % (dbA, BODY_R))
    print('  %-22s %6s | %-30s | %-30s' % ('目标(穿过最窄缝)', '缝宽', '判据B: 可达/最小柱距', '判据A: 可达/最小柱距'))
    for tag, gap, tx, ty in targets:
        iB, jB = to_idx(tx, ty, freeB)
        iA, jA = to_idx(tx, ty, freeA)
        vB, vA = bestB[iB, jB], bestA[iA, jA]
        sB = '不可达' if vB < 0 else '可达 / %.3f m %s' % (
            vB, '安全' if vB >= BODY_R else '机体撞!')
        sA = '不可达' if vA < 0 else '可达 / %.3f m %s' % (
            vA, '安全' if vA >= BODY_R else '机体撞!')
        print('  %-22s %6s | %-30s | %-30s' % (tag, '%.2f' % gap if gap > 0 else '-', sB, sA))

    nfree = int(freeB.sum())
    safe = int((bestB >= BODY_R).sum())
    print('  全图: 判据B 自由格 %d (%.1f%%); 其中"最优路径下机体全程不擦碰"的格 %d (%.1f%%)'
          % (nfree, 100.0 * nfree / (NX * NY), safe, 100.0 * safe / max(nfree, 1)))
