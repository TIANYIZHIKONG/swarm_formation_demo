#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成 formation.world 的圆柱障碍物区块 (多机编队场地).

为什么用脚本而不是手写?
----------------------
手工摆 40+ 根柱子没法保证下面两件事:

  1) 缝的宽度可控.
     机体含桨尖约 0.7 m 宽 (机身 0.47 + 桨尖外伸 0.348, 见
     config/normal_hexagon.yaml 的注释), 而 ego_planner 的
     `grid_map/obstacles_inflation` 只有 0.2 m/侧, 轨迹优化的软代价
     `optimization/obstacle_clearance=0.5` 是**从膨胀后的面量起**的
     (=> 真实距离 0.7 m 以内就激活, 即缝宽 < 1.4 m 时缝中间就被代价压住).
     本脚本默认 --min-gap 1.2 m, 和上游 swarm.launch 里 random_forest 的
     `min_distance=1.2` 对齐 (缝里会激活软代价 -> 飞得更慢更绕, 但过得去);
     想宽松一点用 --min-gap 1.6 (缝中间留出 0.2 m 的"零代价带").
     脚本**保证**全场最小表面缝 = --min-gap, 不靠运气.

  2) 起飞点不被堵.
     第 i 架无人机在 (start_x, start_y + spacing*i) **原地起飞**, 出生点
     若插在柱子里 Gazebo 会直接把它弹飞. 本脚本把 SPAWN_CLEAR 矩形整个
     挖空 (默认够 5 架: y = 0,2,4,6,8); 再额外留一块 PLAZAS 集结空地,
     因为按最小缝塞满之后, 编队 (3 架就要 3.5x4.7 m) 没地方整队落下.

布局算法 = 抖动网格 + 密度场 + 半径自适应:
  * 候选点先按全场抖动网格铺一遍 (不会出现整条空带), 再撒一批随机飞镖补缝;
  * 每个候选点的"留空概率"由密度场 (DENSITY_BLOBS) 决定 -> 有疏有密;
  * 目标半径随机, 且大半径的候选点**先放**, 小半径的去填缝 -> 同样面积
    能塞得最多, 粗细也不齐 (实测: 先小后大会少掉三分之一);
  * 每个候选点都要过 [场内边界 / 起飞区与集结空地 / 与已放置柱子的最小缝]
    三道检查, 所以"更密"是**在可飞行的前提下**尽量密, 而不是随机乱丢.

用法
----
    cd TECH/sim_assets
    # 重新生成 (默认参数, 原地覆盖 worlds/formation.world)
    python3 scripts/gen_formation_world.py

    # 换布局/加密度: 越小越密 (1.0 已经很极限), 也越难飞
    python3 scripts/gen_formation_world.py --seed 42 --min-gap 1.0

    # 只看不写: 打印柱子数 + 最小缝 + ASCII 俯视图
    python3 scripts/gen_formation_world.py --dry-run

    如果不用默认路径 (worlds/formation.world):
    python3 scripts/gen_formation_world.py --world /path/to/x.world --out /tmp/x.world

脚本只替换 world 文件里 OBSTACLES:BEGIN 与 OBSTACLES:END 两个标记之间的
内容, physics / magnetic_field / gui / include 等**一个字都不动**
(magnetic_field 少一个, PX4 就报 "Yaw estimate error" 拒绝解锁).
world 文件里没有标记时, 区块插到 </world> 之前.

两个容易踩的地方 (脚本里都做了处理):
  * 标记之外的**手写柱子** (先前的 4 根 pillar_p8/pm8/w6/wm6) 是读 world 文件
    来的, 不会重复写一遍, 但会参与避让/最小缝检查 -> 改那几根柱子不用改脚本;
  * 写盘前会把"区块 + 文件原有内容"合起来做一次自检 (XML 良构 / 柱子不重名 /
    最小表面缝 / 起飞区与集结空地仍空 / <magnetic_field> 还在), 任何一条不过
    就**拒绝写文件**并报原因 —— 宁可生成失败, 也不要写一个 Gazebo 加载不了的
    world (注释里出现连续两个减号就会这样).
"""

import argparse
import math
import os
import random
import re
import sys
import xml.etree.ElementTree as ET

BEGIN_MARK = "OBSTACLES:BEGIN"
END_MARK = "OBSTACLES:END"

# --------------------------------------------------------------------------
#  场地参数 (改这里就能换布局)
# --------------------------------------------------------------------------

# 柱子中心允许出现的范围. 规划器的 map_size_x/y = 40/30, 而地图缓冲区是以
# 无人机当前位置为中心的, 起飞点在 x≈0, 所以 |x| <= 13 / |y| <= 11 都在
# 地图里, 还留了余量.
FIELD = (-13.0, -11.0, 13.0, 11.0)          # xmin ymin xmax ymax

# 起飞区: 必须完全空. 覆盖 5 架 (y = 0,2,4,6,8) + 起降余量.
# 柱子中心到该矩形的最短距离必须 >= 半径 + SPAWN_MARGIN.
# 矩形窄一点 = 场地中间能多放柱子; 只要"起飞那条线"足够空就行.
SPAWN_CLEAR = (-1.8, -1.8, 1.8, 9.8)
SPAWN_MARGIN = 1.0

# 编队集结空地 (允许留白的矩形; 柱子中心到矩形距离 >= 半径 + PLAZA_MARGIN).
# 为什么必须留? 编队阵位占的地方不小 (swarm_scale=2.0, 阵位取自
# config/normal_hexagon.yaml, 加机体 0.7 m 余量后): 3 架约 4.2x4.7 m,
# 5 架约 7.6x6.7 m. 场地若按最小缝塞满, 编队**没有任何地方能整队落下去**,
# FSM 会一直在"编队未到位"里打转. 默认在起飞点正东留 9x8 m 空地:
# 在 RViz 点这块区域当编队中心, 起飞 -> 东侧集结; 其余区域仍是密林.
# 注意: 7 架要 7.6x8.7 m, 这块空地(8 m 高)装不下 —— 本仓库 formation.launch
# 的 group 只开到 num_drones=5, 真要上 7 架得把 PLAZAS 的 y 范围放大 (会少几根柱子).
PLAZAS = [
    ("P_集结空地", 3.6, -3.4, 12.6, 4.6),
]
PLAZA_MARGIN = 1.0

# 原有的 4 根柱子 (改动前就在 world 里, 这里原样保留并参与避让).
FIXED_PILLARS = [
    ("pillar_p8",  4.0,  8.0, 0.40),
    ("pillar_pm8", 4.0, -8.0, 0.40),
    ("pillar_w6", -8.0,  6.0, 0.40),
    ("pillar_wm6", -8.0, -6.0, 0.40),
]

# 密度场: (名字, 中心x, 中心y, sigma, 强度). 这些地方柱子更密、更粗; 远离它们的
# 地方会随机留空 -> 形成"聚簇 + 空地"的不规律感, 而不是均匀网格.
DENSITY_BLOBS = [
    ("A_东北",    7.0,  5.2, 3.0, 0.95),
    ("B_西南",   -5.9, -6.4, 2.8, 0.95),
    ("C_东南",    9.8, -5.4, 2.8, 0.90),
    ("D_西",    -10.6,  2.2, 2.6, 0.90),
    ("E_东侧带",  5.1,  0.2, 2.4, 0.85),
    ("F_西北",   -4.6,  8.9, 2.2, 0.80),
    ("G_东北角", 11.3,  9.1, 1.8, 0.80),
    ("H_南侧带",  0.0, -9.4, 2.6, 0.85),
    ("I_东北近",  5.0,  9.2, 1.8, 0.80),
    ("K_西侧带", -6.3, -0.4, 2.4, 0.85),
    ("L_南近",    0.0, -5.8, 2.2, 0.80),
]

# 候选点 = 全场抖动网格 (间距 SPACING, 每轴抖 ±JITTER) + 若干随机飞镖 (补缝).
# 网格保证"到处都有候选", 不会有整条空带; 抖动 + 密度场 + 随机跳过保证不规律.
#
# 几何关系 (很重要, 改参数时照这个来):
#   两根柱子要满足 表面缝 >= min_gap, 即 中心距 >= r1 + r2 + min_gap.
#   网格间距 s、典型半径 r0、抖动 j 三者要满足  s >= 2*(r0 + j) + min_gap,
#   否则相邻两个候选点会互相挤掉 (实测能少掉一半柱子).
#   默认: r0=0.40, j=0.15, min_gap=1.20 -> s = 2.30.
R0        = 0.40   # m, 典型柱子半径 (最粗的那批叫"大树", 见 P_BIG)
JITTER    = 0.15   # m, 每轴抖动幅度
# 基础占位概率 (再小就出现大片空地, 离"更密"更远). 注意: 实测它**不是数量旋钮** ——
# 柱子数其实被 min_gap 的 packing 极限卡住 (min_gap=1.2 时全场约 73 根, 提到 0.85/0.95
# 反而只有 65/66 根, 因为候选点互相挤); 它决定的是"疏密分布", 即哪几条缝宽、哪几片留空.
P_BASE    = 0.72
P_BIG     = 0.08   # "大树"比例: 目标半径取到 [0.5*R_MAX, R_MAX], 会吃掉周围空间
                   #   -> 天然留出空地, 也让场地看起来更不规律
# 额外的随机候选点 (打破网格感, 并用细柱子把缝塞满). 数量不是越多越好:
# 实测 4000 个飞镖不会多出柱子 (格子已经满了), 只是把细缝换成更粗的柱子 (覆盖率 7.4% -> 9.7%).
DARTS     = 1500
R_MIN     = 0.25   # m, 普通柱子目标半径下限
R_MAX     = 0.80   # m, 半径上限 (只对"大树"生效)
R_ACCEPT  = 0.20   # m, 放不下这么大的半径就放弃候选点 (放宽一点可以多塞细柱子)

# 网格间距不去手填, 直接用上面的几何关系推出来: s = 2*(r0 + jitter) + min_gap.
# (用 --min-gap 改了缝, 间距会自动跟着变; 也可以用 --spacing 硬指定.)
SPACING_MIN_GAP_REF = 1.2
SPACING   = 2.0 * (R0 + JITTER) + SPACING_MIN_GAP_REF

# 柱子的高度: 都高于飞行高度 (飞 z≈0.5~1.5), 所以整个场地是"二维迷宫";
# 高度略有不齐是为了看起来不规则.
#
# !!! 改这里之前先看下面这条感知约束 (2026-10-13 第四轮踩过的坑) !!!
#   柱高必须 > 飞行带上界 + 余量, 但**光靠"高于飞行高度"是不够的**: 规划器的
#   地图里柱子能建到多高, 受两条硬限制:
#     1) LiDAR 垂直 FOV 只有 ±15° (16 线) -> 上限 = z_雷达 + d*tan15° = +0.268d;
#     2) grid_map/max_ray_length = 4.5 m 之外的点只做清障、不记为障碍。
#   => 地图里的柱高上限 ≈ (z_机 + 0.20) + 4.5*tan15° = z_机 + 1.41 m。
#   所以柱顶以上 0.6~1.5 m 在地图里是"假空白", A* 会选"从上方绕"的高路 ->
#   真实世界里撞在柱身上部。必须靠 advanced_param.xml 的 virtual_ceil_height
#   (默认 2.0 m, 见该文件注释) 把飞行高度带压到"地图可靠建高区"以内,
#   让柱子在可行高度带内处处实心, 才能真正得到二维迷宫。
#   验证: python3 sim_assets/scripts/diag/height_probe.py 5 0
LEN_RANGE = (2.5, 3.5)

# world 文件里 pose/radius/length 只写到 3 位小数, 从文件反算几何量时的容差.
ROUND_TOL = 2e-3


# --------------------------------------------------------------------------
#  几何检查
# --------------------------------------------------------------------------
def rect_clearance(x, y, rect):
    """点 (x,y) 到矩形 rect 的最短距离; 点在矩形内时返回 0."""
    xmin, ymin, xmax, ymax = rect
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    return math.hypot(dx, dy)


def inside(x, y, rect):
    xmin, ymin, xmax, ymax = rect
    return xmin <= x <= xmax and ymin <= y <= ymax


def surface_gap(a, b):
    """两个圆柱的**表面**最短距离 (中心距 - 两半径). 负数 = 相交."""
    return math.hypot(a[0] - b[0], a[1] - b[1]) - a[2] - b[2]


def min_gap_of(obstacles):
    """全场最小表面缝 (只和最近的邻居比, 数量小就直接两两算)."""
    best = float("inf")
    for i in range(len(obstacles)):
        for j in range(i + 1, len(obstacles)):
            g = surface_gap(obstacles[i], obstacles[j])
            if g < best:
                best = g
    return best


# --------------------------------------------------------------------------
#  生成
# --------------------------------------------------------------------------
def density(x, y, blobs=None):
    """密度场: 0 = 最疏 (会随机留空), 1 = 最密 (几乎每个候选点都放)."""
    d = 0.0
    for (_n, cx, cy, sigma, strength) in (blobs if blobs is not None else DENSITY_BLOBS):
        dd = ((x - cx) ** 2 + (y - cy) ** 2) / (2.0 * sigma * sigma)
        if dd < 30.0:                    # exp(-30) 已经可以忽略
            d = max(d, strength * math.exp(-dd))
    return min(d, 1.0)


def fit_radius(x, y, rmax, min_gap, placed, keep_outs):
    """在 (x,y) 上"能放下的最大半径".

    半径同时受三件事夹住: 场边界 / 起飞区与集结空地 / 与已有柱子的最小缝.
    夹出来比 R_ACCEPT 还小就返回 None —— 这种位置宁可留空, 不然会塞满细柱子,
    场地看起来是一层毛, 也不好飞.
    """
    r = rmax
    r = min(r, x - FIELD[0], FIELD[2] - x, y - FIELD[1], FIELD[3] - y)
    if r < R_ACCEPT:
        return None
    for (rect, margin) in keep_outs:
        r = min(r, rect_clearance(x, y, rect) - margin)
        if r < R_ACCEPT:
            return None
    # 原本想用距离排序做剪枝, 但柱子总数只有百来根, 全扫一遍更简单可靠.
    for (ox, oy, orad) in placed:
        d = math.hypot(x - ox, y - oy) - orad - min_gap
        if d < r:
            r = d
            if r < R_ACCEPT:
                return None
    return r


def build(seed, min_gap, spacing, jitter, darts, big_frac=None, r_max=None, blobs=None,
          fixed=None):
    """返回 (new_obstacles, all_obstacles, stats).

    new_obstacles 每项 (x, y, radius, length) —— 只有**新生成**的柱子, 已有
    (手写) 的柱子不在里面, 因为它们本来就在 world 文件里, 不能重复写一遍;
    all_obstacles 每项 (x, y, radius), 含 fixed 那几根 (用于避让与统计).
    fixed 默认取脚本里的 FIXED_PILLARS; 传 None 之外的值时以传入的为准
    (main 会从 world 文件里读, 免得两边不一致).
    """
    big_frac = P_BIG if big_frac is None else big_frac
    r_max = R_MAX if r_max is None else r_max
    fixed = FIXED_PILLARS if fixed is None else fixed
    rng = random.Random(seed)
    placed = [(x, y, r) for (_n, x, y, r) in fixed]
    new = []
    keep_outs = [(SPAWN_CLEAR, SPAWN_MARGIN)]
    for (_pname, xmin, ymin, xmax, ymax) in PLAZAS:
        keep_outs.append(((xmin, ymin, xmax, ymax), PLAZA_MARGIN))

    # ---- 候选点 1: 抖动网格 (居中排布, 免得只在场地一侧留边) ----
    nx = max(1, int(math.floor((FIELD[2] - FIELD[0]) / spacing)))
    ny = max(1, int(math.floor((FIELD[3] - FIELD[1]) / spacing)))
    ox = FIELD[0] + ((FIELD[2] - FIELD[0]) - nx * spacing) / 2.0
    oy = FIELD[1] + ((FIELD[3] - FIELD[1]) - ny * spacing) / 2.0
    cands = []
    for iy in range(ny):
        for ix in range(nx):
            cands.append((ox + (ix + 0.5) * spacing, oy + (iy + 0.5) * spacing))
    rng.shuffle(cands)                    # 打乱顺序: 先放的柱子会把后来的挤细
    # ---- 候选点 2: 随机飞镖, 专门去补网格之间的洞 ----
    for _ in range(max(0, darts)):
        cands.append((rng.uniform(FIELD[0], FIELD[2]), rng.uniform(FIELD[1], FIELD[3])))

    stats = {"候选点": len(cands), "跳过(留空)": 0, "放不下": 0, "大树": 0}
    # 先"做计划": 每个候选点抽好目标半径; 再按半径从大到小排序后才真正放置.
    # 顺序很关键 —— 大的先占地方, 小的去填缝, 同样面积能放下的柱子最多;
    # 反过来 (先小后大) 大柱子就没位置了, 实测能少掉三分之一.
    plan = []
    for (cx, cy) in cands:
        x = cx + rng.uniform(-jitter, jitter)
        y = cy + rng.uniform(-jitter, jitter)
        d = density(x, y, blobs)
        # 密度场只决定"这里留不留空": 密集处几乎必放, 稀疏处按概率空着.
        if rng.random() > P_BASE + (1.0 - P_BASE) * d:
            stats["跳过(留空)"] += 1
            continue
        if rng.random() < big_frac:
            rmax = rng.uniform(0.5 * r_max, r_max)       # 大树
            stats["大树"] += 1
        else:
            rmax = rng.uniform(R_MIN, R0)
        plan.append((rmax, x, y))
    plan.sort(key=lambda t: (-t[0], t[1], t[2]))         # 同半径用坐标定序, 保证可复现

    for (rmax, x, y) in plan:
        # fit_radius 再按实际空隙夹小 -> 粗细不齐 (粗的落在空处, 细的挤在缝里).
        r = fit_radius(x, y, rmax, min_gap, placed, keep_outs)
        if r is None:
            stats["放不下"] += 1
            continue
        length = rng.uniform(*LEN_RANGE)
        placed.append((x, y, r))
        new.append((x, y, r, length))
    return new, placed, stats


# --------------------------------------------------------------------------
#  渲染 SDF
# --------------------------------------------------------------------------
PILLAR_TMPL = """    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{r:.3f}</radius><length>{l:.3f}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{r:.3f}</radius><length>{l:.3f}</length></cylinder></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>
"""


def render_pillar(name, x, y, r, length):
    # 圆柱 SDF 的 pose 在**几何中心**, 所以要抬高 length/2 让它站在地面上
    return PILLAR_TMPL.format(name=name, x=x, y=y, z=length / 2.0, r=r, l=length)


def ascii_map(obstacles, cell=1.0):
    """把柱子投影成俯视 ASCII 图 (每 cell 米一个字符), 方便肉眼检查布局."""
    x0, y0, x1, y1 = FIELD
    nx = int(round((x1 - x0) / cell))
    ny = int(round((y1 - y0) / cell))
    grid = [[" "] * nx for _ in range(ny)]
    # 集结空地 (留白矩形) 与起飞区 (挖空的矩形)
    for (_pname, xmin, ymin, xmax, ymax) in PLAZAS:
        for iy in range(ny):
            for ix in range(nx):
                cx = x0 + (ix + 0.5) * cell
                cy = y0 + (iy + 0.5) * cell
                if inside(cx, cy, (xmin, ymin, xmax, ymax)):
                    grid[iy][ix] = "+"
    for iy in range(ny):
        for ix in range(nx):
            cx = x0 + (ix + 0.5) * cell
            cy = y0 + (iy + 0.5) * cell
            if inside(cx, cy, SPAWN_CLEAR):
                grid[iy][ix] = ":"
    for (x, y, r) in obstacles:
        ix = int((x - x0) / cell)
        iy = int((y - y0) / cell)
        if 0 <= ix < nx and 0 <= iy < ny:
            grid[iy][ix] = "O" if r >= 0.5 else "o"
    # 从上往下打印 (行 = +y -> -y). y 轴标注用整数米.
    lines = ["       " + "".join("|" if ix % 5 == 0 else "-" for ix in range(nx))]
    for iy in range(ny - 1, -1, -1):
        cy = y0 + iy + 0.5
        label = "%+4d " % int(round(cy)) if abs(round(cy) - cy) < 1e-6 else "     "
        lines.append("%s %s" % (label, "".join(grid[iy])))
    lines.append("       " + "".join("|" if ix % 5 == 0 else "-" for ix in range(nx)))
    xticks = "       "
    for ix in range(nx):
        cx = x0 + ix + 0.5
        xticks += "%d" % (abs(int(round(cx))) % 10) if abs(round(cx) - cx) < 1e-6 else " "
    lines.append(xticks + "      (x 轴, 每格 1 m; O=半径>=0.5, o=更小, :=起飞区, +=集结空地)")
    return "\n".join(lines)


def xml_safe(s):
    """XML 注释里不允许出现连续两个减号, 也不允许以减号结尾 (本仓库 world 文件里
    那条"注释里不能出现连续两个减号"的说明就是这个坑). 生成注释前一律过一遍."""
    while "--" in s:
        s = s.replace("--", "-")
    return s.rstrip("-")


def fmt_rect(rect):
    """把 (xmin, ymin, xmax, ymax) 写成读得懂的 x[...] y[...] (注释里用)."""
    xmin, ymin, xmax, ymax = rect
    return "x[%.1f,%.1f] y[%.1f,%.1f]" % (xmin, xmax, ymin, ymax)


def render_block(seed, min_gap, new, all_obs, stats, param_line):
    """生成要写进 world 的整段障碍物区块 (含标记注释).

    param_line 只放 "k=v" 形式的参数: XML 注释里不允许连续两个减号, 所以
    不能把带 --seed 这种双减号长选项的命令原样写进去 (写进去 world 就坏了,
    Gazebo 直接加载失败). 完整命令会打在脚本的 stdout 里.
    """
    parts = []
    parts.append("    <!-- ==================== %s (本区块由 scripts/gen_formation_world.py 生成, 不要手改) ==================== -->"
                 % BEGIN_MARK)
    parts.append("    <!-- 生成参数: %s (完整命令见生成时的 stdout) -->" % xml_safe(param_line))
    parts.append("    <!-- 参数: seed=%d  min_gap=%.2f m  柱子数=%d (本区块新增 %d + 区块外原有 %d) -->"
                 % (seed, min_gap, len(all_obs), len(new), len(all_obs) - len(new)))
    parts.append("    <!-- 候选点统计: %s -->"
                 % "  ".join("%s=%d" % (k, v) for k, v in stats.items()))
    parts.append("    <!-- 半径: 最小 %.2f m / 平均 %.2f m / 最大 %.2f m ; 障碍占地覆盖率 %.1f%% -->"
                 % (min(o[2] for o in all_obs), sum(o[2] for o in all_obs) / len(all_obs),
                    max(o[2] for o in all_obs),
                    100.0 * sum(math.pi * o[2] ** 2 for o in all_obs) /
                    ((FIELD[2] - FIELD[0]) * (FIELD[3] - FIELD[1]))))
    parts.append("    <!-- 全场最小表面缝 = %.3f m ; 可飞临界 1.4 m = 膨胀 0.2x2 + 机体宽 0.7 + 余量 -->"
                 % min_gap_of(all_obs))
    parts.append("    <!-- 布局: 抖动网格 + 密度场随机留空 (聚簇处更密更粗); 原起飞区 %s 与集结空地 %s 已挖空 -->"
                 % (fmt_rect(SPAWN_CLEAR),
                    "; ".join("%s %s" % (p[0], fmt_rect(p[1:])) for p in PLAZAS)))
    parts.append("    <!-- 想换布局或看俯视图, 重跑 scripts/gen_formation_world.py 即可 (dry run 只看不写) -->")
    parts.append("")
    parts.append("    <!-- 区块外原有 %d 根柱子 (pillar_p8/pm8/w6/wm6, 在上面的手写区里) 原样保留, "
                 "它们已经算进全场最小缝和避让保证了, 这里不再重复写一遍 -->"
                 % (len(all_obs) - len(new)))
    parts.append("    <!-- 以下 %d 根新柱子: 抖动网格铺底 + 密度场随机留空 + 随机飞镖填缝 -->"
                 % len(new))
    for i, (x, y, r, length) in enumerate(new):
        parts.append(render_pillar("pillar_g%03d" % i, x, y, r, length))
    parts.append("    <!-- ==================== %s ==================== -->" % END_MARK)
    return "\n".join(parts) + "\n"


# --------------------------------------------------------------------------
#  写回 world 文件
# --------------------------------------------------------------------------
def block_pattern():
    """匹配 BEGIN/END 标记之间那一整段 (含两个标记注释)."""
    return re.compile(
        r"[ \t]*<!--[^>]*" + re.escape(BEGIN_MARK) + r"[^>]*-->.*?"
        r"<!--[^>]*" + re.escape(END_MARK) + r"[^>]*-->[ \t]*\n",
        re.S)


def patch_world(text, block):
    """把 block 替换到 BEGIN/END 标记之间; 没有标记就插到 </world> 之前."""
    pat = block_pattern()
    if BEGIN_MARK in text and END_MARK in text:
        if not pat.search(text):
            raise SystemExit("[gen_formation_world] 找到标记但无法匹配区块, 请手查该 world 文件")
        return pat.sub(lambda _m: block, text, count=1)
    idx = text.rfind("</world>")
    if idx < 0:
        raise SystemExit("[gen_formation_world] 这不是一个 world 文件 (没有 </world>)")
    # 插到 </world> 所在行的**行首**, 不要插在缩进和 </world> 之间 (那样会把
    # </world> 的缩进吃掉, 文件尾部看起来脏).
    line_start = text.rfind("\n", 0, idx) + 1
    return text[:line_start] + block + text[line_start:]


def strip_generated_block(text):
    """去掉已生成的障碍物区块, 只留文件原本的内容 (标记之前/之后的手写部分)."""
    pat = block_pattern()
    if BEGIN_MARK in text and END_MARK in text and pat.search(text):
        return pat.sub("", text, count=1)
    return text


def collect_pillars(text):
    """从 world 文本里读出所有 pillar_* 圆柱 -> [(name, x, y, r)] (按名字排序).

    顺便验证 XML 良构: 生成时注释里若混进非法字符 (比如 XML 注释里不能出现的
    连续两个减号), 这里当场就报出来, 不至于写进 world 之后 Gazebo 加载失败.
    """
    try:
        root = ET.fromstring(text)
    except ET.ParseError as e:
        raise SystemExit("[gen_formation_world] world 不是合法 XML: %s "
                         "(多半是注释里混进了非法字符)" % e)
    out = []
    for w in root.findall("world"):
        for m in w.findall("model"):
            name = m.get("name") or ""
            if not name.startswith("pillar_"):
                continue
            try:
                pose = [float(v) for v in m.find("pose").text.split()]
                cyl = m.find("link/collision/geometry/cylinder")
                r = float(cyl.find("radius").text)
            except (AttributeError, ValueError) as e:
                raise SystemExit("[gen_formation_world] 柱子 %s 结构不对: %s" % (name, e))
            out.append((name, pose[0], pose[1], r))
    return sorted(out)


def verify_world(text, min_gap):
    """写盘前的最后一道闸 (生成的区块 + 文件原有内容一起查).

    查的都是"写错了就飞不起来"的项: XML 良构 / 柱子不重名 / 最小表面缝 /
    起飞区与集结空地挖空 / <magnetic_field> 还在. 任何一条不过就拒绝写文件.
    """
    pillars = collect_pillars(text)
    names = [p[0] for p in pillars]
    dup = sorted({n for n in names if names.count(n) > 1})
    if dup:
        raise SystemExit("[gen_formation_world] 柱子重名 (Gazebo 会错位): %s" % ", ".join(dup))
    obs = [(x, y, r) for (_n, x, y, r) in pillars]

    # world 文件里坐标/半径只保留 3 位小数, 所以从文件里反算出来的缝会比生成时
    # 小一点点 (实测 1.200 -> 1.199). 2 mm 的容差足够, 也远小于可飞临界 1.4 m,
    # 不会掩盖"真的撞在一起"的情况.
    gap = min_gap_of(obs)
    if gap < min_gap - ROUND_TOL:
        raise SystemExit("[gen_formation_world] 全场最小表面缝 %.3f m < 要求 %.2f m"
                         % (gap, min_gap))

    keep_outs = [(SPAWN_CLEAR, SPAWN_MARGIN)] + \
        [((p[1], p[2], p[3], p[4]), PLAZA_MARGIN) for p in PLAZAS]
    for (rect, margin) in keep_outs:
        d = min(rect_clearance(x, y, rect) - r for (x, y, r) in obs)
        if d < margin - ROUND_TOL:            # 同样放宽 2 mm 的取整误差
            raise SystemExit("[gen_formation_world] 有柱子挤进挖空区 %s (余量 %.3f < %.2f)"
                             % (fmt_rect(rect), d, margin))

    if not re.search(r"<magnetic_field[ >]", text):
        raise SystemExit("[gen_formation_world] world 里没有 <magnetic_field>, PX4 会报 "
                         "\"Yaw estimate error\" 拒绝解锁, 先补回来再生成")
    return len(pillars), gap


# --------------------------------------------------------------------------
#  main
# --------------------------------------------------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_world = os.path.join(os.path.dirname(here), "worlds", "formation.world")

    ap = argparse.ArgumentParser(
        description="生成 formation.world 的圆柱障碍物区块 (抖动网格 + 密度场留空, "
                    "保证最小表面缝与起飞区/集结空地不被堵)")
    ap.add_argument("--world", default=default_world,
                    help="要改的 world 文件 (默认 %(default)s)")
    ap.add_argument("--out", default=None, help="输出文件 (默认原地覆盖 --world)")
    ap.add_argument("--seed", type=int, default=20261013, help="随机种子 (换布局)")
    ap.add_argument("--min-gap", type=float, default=1.2,
                    help="两根柱子之间的最小表面缝, 单位 m (默认 %(default)s; "
                         "对齐上游 swarm.launch 里 random_forest 的 min_distance=1.2. "
                         "1.4 是 EGO 软代价 (obstacle_clearance=0.5, 膨胀 0.2) 刚好不激活"
                         "的中心线余量, 缝更小 -> 代价在每条缝里都激活, 飞得更慢更绕; "
                         "想要宽松点用 1.6)")
    ap.add_argument("--spacing", type=float, default=None,
                    help="候选点网格间距 m (默认按几何关系自动推: 2*(R0+jitter)+min_gap)")
    ap.add_argument("--jitter", type=float, default=JITTER,
                    help="网格抖动幅度 m (默认 %(default)s)")
    ap.add_argument("--darts", type=int, default=DARTS,
                    help="额外随机候选点数 (补网格缝, 默认 %(default)s)")
    ap.add_argument("--big-frac", type=float, default=P_BIG,
                    help="\"大树\"比例 (目标半径到 --rmax, 会吃掉周围空间, 默认 %(default)s)")
    ap.add_argument("--rmax", type=float, default=R_MAX,
                    help="柱子半径上限 m (默认 %(default)s)")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印统计和 ASCII 俯视图, 不写文件")
    ap.add_argument("--print-block", action="store_true", help="把生成的区块打到 stdout")
    args = ap.parse_args()

    if not os.path.isfile(args.world):
        raise SystemExit("[gen_formation_world] 找不到 world 文件: %s" % args.world)

    with open(args.world, "r", encoding="utf-8") as f:
        text = f.read()

    # 已有的柱子以 **world 文件** 为准 (区块之外的手写部分), 不重写进区块,
    # 但必须参与避让: 少了它们, 生成的柱子会直接插到原有柱子上.
    existing = collect_pillars(strip_generated_block(text))
    expected = sorted(FIXED_PILLARS)
    if not existing:
        print("[gen_formation_world] 注意: world 里没找到已有的 pillar_*, "
              "改用脚本里写死的 FIXED_PILLARS (%d 根)" % len(expected))
        existing = expected
    elif [(n, round(x, 3), round(y, 3), round(r, 3)) for (n, x, y, r) in existing] != \
         [(n, round(x, 3), round(y, 3), round(r, 3)) for (n, x, y, r) in expected]:
        print("[gen_formation_world] 注意: world 里已有的柱子 (%s) 和脚本里的 "
              "FIXED_PILLARS 不一致, 以 world 文件为准"
              % ", ".join(p[0] for p in existing))

    spacing = args.spacing if args.spacing else 2.0 * (R0 + args.jitter) + args.min_gap
    new, all_obs, stats = build(args.seed, args.min_gap, spacing, args.jitter,
                                args.darts, args.big_frac, args.rmax, fixed=existing)
    if not new:
        raise SystemExit("[gen_formation_world] 一根柱子都没放下, 检查 --min-gap / --spacing")

    param_line = ("seed=%d  min_gap=%g  jitter=%g  darts=%d  big_frac=%g  rmax=%g  spacing=%.2f"
                  % (args.seed, args.min_gap, args.jitter, args.darts,
                     args.big_frac, args.rmax, spacing))
    block = render_block(args.seed, args.min_gap, new, all_obs, stats, param_line)

    gen_cmd = ("python3 scripts/gen_formation_world.py --seed %d --min-gap %g --jitter %g"
               " --darts %d --big-frac %g --rmax %g"
               % (args.seed, args.min_gap, args.jitter, args.darts, args.big_frac, args.rmax))
    print("[gen_formation_world] 生成命令(可复现同一布局): %s" % gen_cmd)

    # -------- 统计 --------
    radii = [o[2] for o in all_obs]
    area = sum(math.pi * r * r for r in radii)
    print("[gen_formation_world] 柱子: 区块外原有 %d + 本区块新增 %d = %d 根"
          % (len(existing), len(new), len(all_obs)))
    print("[gen_formation_world] 候选点: %s"
          % "  ".join("%s=%d" % (k, v) for k, v in stats.items()))
    print("[gen_formation_world] 半径: min %.2f / 平均 %.2f / max %.2f m ; 占地覆盖率 %.1f%%"
          % (min(radii), sum(radii) / len(radii), max(radii),
             100.0 * area / ((FIELD[2] - FIELD[0]) * (FIELD[3] - FIELD[1]))))
    print("[gen_formation_world] 全场最小表面缝 = %.3f m (要求 >= %.2f)"
          % (min_gap_of(all_obs), args.min_gap))
    near = min((rect_clearance(x, y, SPAWN_CLEAR) - r) for (x, y, r) in all_obs)
    print("[gen_formation_world] 柱子表面到起飞区矩形的最近距离 = %.3f m" % near)
    for (pname, xmin, ymin, xmax, ymax) in PLAZAS:
        nearp = min((rect_clearance(x, y, (xmin, ymin, xmax, ymax)) - r)
                    for (x, y, r) in all_obs)
        print("[gen_formation_world] 柱子表面到集结空地 %s 的最近距离 = %.3f m"
              % (pname, nearp))
    xs = [o[0] for o in all_obs]
    ys = [o[1] for o in all_obs]
    print("[gen_formation_world] 包围盒: x[%.2f, %.2f] y[%.2f, %.2f]"
          % (min(xs), max(xs), min(ys), max(ys)))
    print("[gen_formation_world] 布局俯视图:")
    print(ascii_map(all_obs))

    if args.print_block:
        print(block)

    if args.dry_run:
        print("[gen_formation_world] --dry-run: 没有写文件")
        return 0

    out = args.out or args.world
    patched = patch_world(text, block)
    n_pillars, gap = verify_world(patched, args.min_gap)
    print("[gen_formation_world] 写盘前自检通过: XML 良构, %d 根柱子不重名, "
          "最小表面缝 %.3f m, 起飞区/集结空地仍空, <magnetic_field> 在"
          % (n_pillars, gap))
    with open(out, "w", encoding="utf-8") as f:
        f.write(patched)
    print("[gen_formation_world] 已写入 %s (%d -> %d 字节)"
          % (out, len(text.encode("utf-8")), len(patched.encode("utf-8"))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
