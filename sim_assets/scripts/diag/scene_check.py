#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene_check.py -- 把"机体卡在哪儿"落到**具体场景物体**上 (柱子/墙/地板/天花板)

为什么需要:
  stuck_probe.py 只说"机体中心落在膨胀障碍格内 -> A* 起点非法"。
  但复现"贴柱飞行卡住"时要回答的是: 到底贴着哪根柱子、表面离多远、
  obstacles_inflation=0.35 m 之后这个点是不是**必然**非法 —— 本脚本一次说清。

做法:
  · 解析 sim_assets/worlds/formation.world 里所有 <model> 的 <pose> + <geometry>
    (cylinder / box / sphere);
  · 取机体当前位置: 默认读 /drone_<ID>_visual_slam/odom, 也可 --at X Y 手给;
    **注意**: odom 与场景 pose **同在 world 系** (vio_offset.py 给 odom 和点云加了
    同一个固定平移), 所以可以直接和场景几何比; Gazebo 真值(model origin)比 odom
    低 mount_z(默认 0.20 m), 那是"模型原点 vs 雷达点"的定义差, 不要混用。
  · 打印最近若干物体的: 名称/类型/尺寸/中心水平距离/**到表面的三维距离**;
  · 判定: 表面距离 < obstacles_inflation => 该点必在膨胀障碍体内部
    => FSM 拿它当 A* 起点 => planFromLocalTraj 必失败 => 撞后卡死。

用法:
  python3 scene_check.py                        # drone_0, 用当前 odom
  python3 scene_check.py 0 --at 5.71 -6.65      # 用卡住时的坐标(用真值 xy 也行)
  python3 scene_check.py 0 --inflation 0.35 --top 8
"""
import argparse
import os

import numpy as np
import xml.etree.ElementTree as ET

WORLD = os.environ.get('SCENE_WORLD') or os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 '..', '..', 'worlds', 'formation.world'))


def load_scene(path):
    """解析 world, 返回每个 model 的名称/几何/中心。"""
    out = []
    if not os.path.exists(path):
        return out, 'world 文件不存在: %s' % path
    root = ET.parse(path).getroot()
    for m in root.iter('model'):
        cen = [0.0, 0.0, 0.0]
        p = m.findtext('pose')
        if p:
            cen = [float(x) for x in p.split()[:3]]
        kind, size = None, {}
        g = m.find('.//geometry')
        if g is not None:
            c, b, s = g.find('cylinder'), g.find('box'), g.find('sphere')
            if c is not None:
                kind = 'cylinder'
                size = {'r': float(c.findtext('radius') or 0),
                        'l': float(c.findtext('length') or 0)}
            elif b is not None:
                kind = 'box'
                sz = (b.findtext('size') or '0 0 0').split()
                size = {'x': float(sz[0]), 'y': float(sz[1]), 'z': float(sz[2])}
            elif s is not None:
                kind = 'sphere'
                size = {'r': float(s.findtext('radius') or 0)}
        out.append({'name': m.get('name', '?'), 'kind': kind, 'size': size,
                    'c': np.array(cen)})
    return out, None


def surf_dist(it, p):
    """点 p 到该几何体表面的三维距离 (体外为正, 体内为 0)。"""
    d = p - it['c']
    k, s = it['kind'], it['size']
    if k == 'cylinder':
        dxy = max(0.0, float(np.linalg.norm(d[:2])) - s['r'])
        dz = max(0.0, abs(float(d[2])) - s['l'] / 2.0)
    elif k == 'box':
        dxy = float(np.linalg.norm(np.maximum(np.abs(d[:2]) -
                                             np.array([s['x'], s['y']]) / 2.0, 0.0)))
        dz = max(0.0, abs(float(d[2])) - s['z'] / 2.0)
    elif k == 'sphere':
        dxy, dz = max(0.0, float(np.linalg.norm(d)) - s['r']), 0.0
    else:                                   # 无 geometry(世界 include / 纯模型)
        dxy, dz = float(np.linalg.norm(d[:2])), abs(float(d[2]))
    return float(np.hypot(dxy, dz))


def live_pos(did):
    """读机体当前位姿 (规划器同系: visual_slam/odom, 退而用 mavros local)。"""
    import rospy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    rospy.init_node('scene_check', anonymous=True)
    for topic, typ in (('/drone_%d_visual_slam/odom' % did, Odometry),
                       ('/drone_%d/mavros/local_position/pose' % did, PoseStamped)):
        try:
            msg = rospy.wait_for_message(topic, typ, 4)
            q = msg.pose.pose.position if hasattr(msg.pose, 'pose') else msg.pose.position
            return np.array([q.x, q.y, q.z]), topic
        except Exception:
            continue
    return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('drone', nargs='?', type=int, default=0)
    ap.add_argument('--at', nargs=2, type=float, default=None, metavar=('X', 'Y'),
                    help='手动给坐标(只需 xy, z 取当前 odom 的 z)')
    ap.add_argument('--inflation', type=float, default=0.35)
    ap.add_argument('--top', type=int, default=6)
    a = ap.parse_args()

    items, err = load_scene(WORLD)
    print('world: %s' % WORLD)
    if err:
        raise SystemExit(err)
    print('解析到 %d 个 model' % len(items))

    if a.at:
        pos, src = np.array([a.at[0], a.at[1], 0.90]), '--at (z 默认 0.90 m)'
        live, _ = live_pos(a.drone)
        if live is not None:
            pos[2] = float(live[2])
            src = '--at xy + 当前 odom z=%.2f' % pos[2]
    else:
        pos, src = live_pos(a.drone)
        if pos is None:
            raise SystemExit('读不到位姿: 仿真在跑吗? 或直接用 --at X Y')
    print('查询点 (world 系) = (%.2f, %.2f, %.2f)   来源: %s'
          % (pos[0], pos[1], pos[2], src))

    ds = sorted(((surf_dist(it, pos), it) for it in items), key=lambda x: x[0])
    print('\n最近的 %d 个场景物体:' % a.top)
    print('  %-16s %-9s %-18s %8s %8s %s'
          % ('名称', '类型', '尺寸', '表面距离', '水平距离', '判定'))
    for sd, it in ds[:a.top]:
        s = it['size']
        if it['kind'] == 'cylinder':
            dim = 'r=%.2f l=%.2f' % (s['r'], s['l'])
        elif it['kind'] == 'box':
            dim = '%.1fx%.1fx%.1f' % (s['x'], s['y'], s['z'])
        elif it['kind'] == 'sphere':
            dim = 'r=%.2f' % s['r']
        else:
            dim = '-'
        dxy = float(np.linalg.norm((pos - it['c'])[:2]))
        if sd < a.inflation:
            tag = '**在膨胀体内 -> A* 起点必非法**'
        elif sd < a.inflation + 0.10:
            tag = '膨胀外缘 (余量 %.2f m)' % (sd - a.inflation)
        else:
            tag = '安全'
        print('  %-16s %-9s %-18s %8.2f %8.2f %s'
              % (it['name'][:16], it['kind'] or '-', dim, sd, dxy, tag))

    sd0, it0 = ds[0]
    print('\n判定: 最近障碍 = %s (%s), 表面距离 %.2f m, obstacles_inflation = %.2f m'
          % (it0['name'], it0['kind'], sd0, a.inflation))
    if sd0 < a.inflation:
        print('  -> 该点在膨胀障碍体内部 (余量 %.2f m < 0).' % (sd0 - a.inflation))
        print('     只要机体停在这里, FSM 的 A* 起点就是非法格 -> planFromLocalTraj 必失败')
        print('     -> 表现为"撞后无法重新规划而卡死" (用 stuck_probe.py 看 FSM 是否在猛试)。')
        print('     对策: 先把机体挪出 %.2f m 再给目标; 或 obstacles_inflation %.2f -> %.2f'
              % (a.inflation - sd0, a.inflation, max(0.15, sd0 - 0.05)))
    else:
        print('  -> 该点不在膨胀体内 (余量 %.2f m): 本次卡住不是"起点非法",'
              ' 请查 fail-safe 是否被重新武装 / 是否被机间避让挡住。' % (sd0 - a.inflation))
        print('     快验: 再发一个 2D Nav Goal, 若收到目标就恢复 -> 就是 fail-safe 未武装。')


if __name__ == '__main__':
    main()
