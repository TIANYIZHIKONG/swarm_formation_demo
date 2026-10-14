#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""静态几何校验: world 里所有柱状障碍之间的"净缝隙"能否被当前算法通过.

各层门槛 (均来自代码, 见下方注释):
  L0 原始柱面缝隙          gap = d_center - r1 - r2
  L1 A*(ESDF 判据)        需要 gap > 2*0.2 = 0.40   [dyn_a_star.cpp:364 AstarSearch(...,true)
                                                     + dyn_a_star.h:65 dist=0.2]
  L2 A*(膨胀判据,回退)    需要 gap > 2*(0.35 + 0.1量化) = 0.90 [obstacles_inflation=0.35,
                                                     resolution=0.1, inf_step=ceil(0.35/0.1)=4]
                                                     (旧值 0.3 时是 0.80, 但那时地图硬边界
                                                      只有 0.316 m < 机体半径 0.348 m)
  L3 优化器(B样条)        控制点间距 0.4 m -> 缝隙最好 > 0.4~0.6 [manager/control_points_distance=0.4]
  L4 机体(含桨尖)不擦碰   需要 gap > 2*0.348 = 0.696   <-- 真正的物理底线
"""
import itertools
import math
import re

W = '/home/safina/keyan_project/something1017/TECH/sim_assets/worlds/formation.world'
FLY_Z = (0.2, 1.8)      # 必须绕行的飞行高度带 (任务里 z=0.5, 机动可能到 1.5+)

txt = open(W).read()
pil = []
for m in re.finditer(r'<model name="(pillar_[^"]+)">(.*?)</model>', txt, re.S):
    name, body = m.group(1), m.group(2)
    pm = re.search(r'<pose>\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', body)
    rm = re.search(r'<cylinder><radius>([\d.]+)</radius><length>([\d.]+)</length>', body)
    if not (pm and rm):
        continue
    x, y, z = float(pm.group(1)), float(pm.group(2)), float(pm.group(3))
    r, h = float(rm.group(1)), float(rm.group(2))
    pil.append(dict(n=name, x=x, y=y, z=z, r=r, h=h, zlo=z - h / 2.0, zhi=z + h / 2.0))

print('柱体数量: %d' % len(pil))
print('半径 min/avg/max = %.3f / %.3f / %.3f m' % (
    min(p['r'] for p in pil), sum(p['r'] for p in pil) / len(pil), max(p['r'] for p in pil)))
print('柱底高度 z_lo: min=%.2f  max=%.2f   (>=0.05 的柱底悬空, 可从下方钻)' % (
    min(p['zlo'] for p in pil), max(p['zlo'] for p in pil)))
print('柱顶高度 z_hi: min=%.2f  max=%.2f   (最小柱顶以下才是"必须绕"的实心区)' % (
    min(p['zhi'] for p in pil), max(p['zhi'] for p in pil)))
print('飞行高度带 = %.1f ~ %.1f m' % FLY_Z)

INFL, RES = 0.35, 0.1                         # = advanced_param.xml grid_map/obstacles_inflation (第三轮)
BODY_R = 0.348
rows = []
for a, b in itertools.combinations(pil, 2):
    d = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
    gap = d - a['r'] - b['r']                          # L0 原始柱面净缝
    overlap_z = min(a['zhi'], b['zhi']) > FLY_Z[0] and max(a['zlo'], b['zlo']) < FLY_Z[1]
    rows.append((gap, d, a, b, overlap_z))
rows.sort(key=lambda t: t[0])

print('\n=== 最窄的 25 对柱间缝隙 (gap = 柱心距 - r1 - r2) ===')
print('%-7s %-7s %8s %7s %7s %7s %8s %9s %9s %9s' %
      ('A', 'B', 'd心距', 'r1', 'r2', 'gap', 'z带重叠', 'A*_esdf', 'A*_infl', '机体'))
for gap, d, a, b, oz in rows[:25]:
    print('%-7s %-7s %8.3f %7.3f %7.3f %7.3f %8s %9s %9s %9s' % (
        a['n'].replace('pillar_', ''), b['n'].replace('pillar_', ''), d, a['r'], b['r'], gap,
        'yes' if oz else 'no',
        'OK' if gap > 0.40 else 'FAIL', 'OK' if gap > 0.90 else 'FAIL',
        'OK' if gap > 2 * BODY_R else '碰'))

z_rows = [t for t in rows if t[4]]
print('\n=== 统计 (只看飞行高度带内重叠的柱对, 共 %d 对) ===' % len(z_rows))
for lim, tag in ((0.40, 'L1 A* ESDF(0.2m)'), (0.70, '  A* 膨胀(0.35m,无量化)'),
                 (0.90, 'L2 A* 膨胀(0.35m+量化)'), (0.696, 'L4 机体不擦碰(2x0.348)')):
    n = sum(1 for g, d, a, b, oz in z_rows if g <= lim)
    print('  gap <= %.3f m (%s): %4d 对 -> %s' % (lim, tag, n, '不够' if n else '全部满足'))
print('  gap  最小 = %.3f m (%s <-> %s)' % (
    z_rows[0][0], z_rows[0][2]['n'], z_rows[0][3]['n']) if z_rows else '')

print('\n=== 无障碍走廊宽度排序(前 12 大) ===')
for gap, d, a, b, oz in z_rows[-12:][::-1]:
    print('  %-28s gap=%.2f m (d=%.2f)' % (a['n'] + ' <-> ' + b['n'], gap, d))
