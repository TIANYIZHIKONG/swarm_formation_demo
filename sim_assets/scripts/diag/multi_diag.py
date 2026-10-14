#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多机体检: 判定"是不是共用 drone0 的点云地图", 并定位哪台机在漂移.

本工程的实际链路 (每机完全独立, 只有轨迹通过 swarm_bridge 共享):
  Gazebo iris_lidar_<i> 的 3D LiDAR /drone_<i>/points (10 Hz, 360deg x +-15deg)
    -> FAST-LIO (每机一个进程)  /drone_<i>/Odometry, /drone_<i>/cloud_registered
    -> vio_offset_drone_<i>  (只做固定平移 -> 公共 world 系)
         /drone_<i>_visual_slam/odom          <- 规划器用的位姿 (grid_map/odom)
         /drone_<i>_pcl_render_node/cloud     <- 规划器用的点云 (grid_map/cloud)
    -> ego_planner (每机一个) 自己建图 / 自己 A* / 自己优化

用法 (先 source sim_assets/env.sh):
  python3 /tmp/multi_diag.py 30 3        # 采样 30 s, 3 架机

判读:
  err_h/err_z 稳定 < 0.15 m   -> 该机定位与 Gazebo 真值对齐, 地图不会整体错位
  err 单调变大 / jump != 0    -> 该机 FAST-LIO 在漂移/跳变, 地图会与真实障碍错位 -> 会撞
  cloud 0 Hz / n=0            -> 该机没有自己的点云 (若真是这样才叫"共用别人的地图")
"""
import math
import sys

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

MOUNT = (0.05, 0.0, 0.20)   # LiDAR/IMU 在机体上的安装偏移 (gen_drone_sdf.py:47)

try:
    DUR = float(sys.argv[1])
except Exception:
    DUR = 10.0
try:
    N = int(sys.argv[2])
except Exception:
    N = 3


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Drone(object):
    def __init__(self, i):
        self.i = i
        self.lio = None        # (x, y, z, yaw) 规划器用的位姿
        self.truth = None      # (x, y, z, yaw) Gazebo 真值
        self.cloud_t = []      # 最近 1 s 的点云时间戳
        self.cloud_n = 0       # 最近一帧点数
        self.last_pos = None
        self.jump = 0.0
        self.err_h = self.err_z = self.err_yaw = float('nan')
        self.lio_n = 0

    def update_err(self):
        if self.lio is None or self.truth is None:
            return
        bx, by, bz, byaw = self.truth
        dx = MOUNT[0] * math.cos(byaw) - MOUNT[1] * math.sin(byaw)
        dy = MOUNT[0] * math.sin(byaw) + MOUNT[1] * math.cos(byaw)
        ex, ey, ez = bx + dx, by + dy, bz + MOUNT[2]
        lx, ly, lz, lyaw = self.lio
        self.err_h = math.hypot(lx - ex, ly - ey)
        self.err_z = lz - ez
        self.err_yaw = math.degrees(wrap(lyaw - byaw))


drones = [Drone(i) for i in range(N)]


def truth_cb(msg):
    for name, pose in zip(msg.name, msg.pose):
        if not name.startswith('iris_lidar_'):
            continue
        try:
            idx = int(name.rsplit('_', 1)[1])
        except ValueError:
            continue
        if 0 <= idx < N:
            p = pose.position
            drones[idx].truth = (p.x, p.y, p.z, yaw_of(pose.orientation))


def make_odom_cb(d):
    def cb(msg):
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        d.lio = (p.x, p.y, p.z, yaw)
        d.lio_n += 1
        if d.last_pos is not None:
            step = math.dist((p.x, p.y, p.z), d.last_pos)
            if step > d.jump:
                d.jump = step
        d.last_pos = (p.x, p.y, p.z)
        d.update_err()
    return cb


def make_cloud_cb(d):
    def cb(msg):
        now = rospy.Time.now().to_sec()
        d.cloud_t.append(now)
        d.cloud_t = [t for t in d.cloud_t if now - t <= 1.0]
        d.cloud_n = msg.width * msg.height
    return cb


rospy.init_node('multi_diag', anonymous=True)
rospy.Subscriber('/gazebo/model_states', ModelStates, truth_cb)
for d in drones:
    rospy.Subscriber('/drone_%d_visual_slam/odom' % d.i, Odometry, make_odom_cb(d))
    rospy.Subscriber('/drone_%d_pcl_render_node/cloud' % d.i, PointCloud2, make_cloud_cb(d))

print('采样 %.0f s ... (Ctrl-C 提前结束)' % DUR)
print('%-6s %-30s %-30s %8s %8s %8s %8s %8s %7s' %
      ('drone', 'LIO(规划用,world)', 'Gazebo真值', 'err_h', 'err_z', 'err_yaw', 'jump', 'cloud', 'pts'))
t_end = rospy.Time.now().to_sec() + DUR
while not rospy.is_shutdown() and rospy.Time.now().to_sec() < t_end:
    rospy.sleep(1.0)
    for d in drones:
        lio = '(%.2f,%.2f,%.2f,y%.0f)' % (d.lio[0], d.lio[1], d.lio[2], math.degrees(d.lio[3])) if d.lio else 'no odom'
        tru = '(%.2f,%.2f,%.2f,y%.0f)' % (d.truth[0], d.truth[1], d.truth[2], math.degrees(d.truth[3])) if d.truth else 'no truth'
        print('%-6s %-30s %-30s %8.3f %8.3f %8.2f %8.3f %6.1fHz %7d' %
              ('d%d' % d.i, lio, tru, d.err_h, d.err_z, d.err_yaw, d.jump,
               len(d.cloud_t), d.cloud_n))

print('\n==== 汇总 ====')
for d in drones:
    flag = []
    if d.lio is None:
        flag.append('没有 odom(!)')
    if d.err_h != d.err_h or d.err_h > 0.3:
        flag.append('水平定位误差大/无真值 -> 地图可能整体错位')
    if d.jump > 0.5:
        flag.append('定位跳变 %.2f m/帧 -> FAST-LIO 异常' % d.jump)
    if len(d.cloud_t) < 3:
        flag.append('自己的点云没有/极慢 -> 该机看不到障碍')
    print('d%d: LIO 帧=%d, cloud=%.1f Hz, 最大跳变=%.3f m, err_h=%.3f m -> %s' %
          (d.i, d.lio_n, len(d.cloud_t), d.jump, d.err_h,
           'OK' if not flag else ' | '.join(flag)))
print('\n若三机的 cloud 都有独立数据且 err 都小而稳定 -> 感知/建图确实是分布式的, 撞障碍来自定位漂移或感知半径不足;')
print('若某机 cloud=0 或 err 持续增大 -> 才是"该机地图不对"的直接证据。')
