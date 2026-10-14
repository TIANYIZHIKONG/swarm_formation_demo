#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""selftest_fake.py -- 不用仿真, 用**假话题**把 diag 脚本的判读链路跑通 (自检用)

!! 警告 !! 它会发布 /drone_<ID>_planning/pos_cmd 之类的**假指令**。
   若真仿真在飞, 控制器会执行这条假指令 (飞机会真的动起来) ——
   **只在仿真没跑时**用它做脚本自检; 完事 pkill -f selftest_fake。

它按 2026-10-14 真实现场的数值造场景:
  /gazebo/model_states        : 机体静止悬停 (默认 5.71,-6.65,0.75), v=0
  /drone_<ID>_visual_slam/odom: = 真值 + (0.011, 0.006, 0.186)   <- 复现实测的坐标口径差
  /drone_<ID>_planning/pos_cmd: traj_id=94 恒定, flag=1(READY), 位置=真值+(0.03,0.01,0)
  .../grid_map/occupancy_inflate: 在 odom 点外 0.011 m 造一列"柱面格"
  .../grid_map/occupancy        : 空云 (复现本工程实测: 该话题不发点)

用法:
  roscore &                        # 已有 master 也可直接用
  python3 selftest_fake.py 0 &
  sleep 1; python3 stuck_probe.py 6 0 1.0 0.05     # 应报出 1 段卡住事件 + d_inf/口径核对
  python3 scene_check.py 0 --at 5.71 -6.65         # 应报 pillar_g072 表面 0.35 m
  pkill -f selftest_fake
"""
import sys

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

ID = int(sys.argv[1]) if len(sys.argv) > 1 else 0
X, Y, Z = 5.71, -6.65, 0.75                     # 与实测卡死位置一致
OFF = np.array([0.011, 0.006, 0.186])           # odom - truth (实测值)

rospy.init_node('selftest_fake', anonymous=True)
p_ms = rospy.Publisher('/gazebo/model_states', ModelStates, queue_size=5)
p_od = rospy.Publisher('/drone_%d_visual_slam/odom' % ID, Odometry, queue_size=5)
p_cm = rospy.Publisher('/drone_%d_planning/pos_cmd' % ID, PositionCommand, queue_size=5)
p_inf = rospy.Publisher('/drone_%d_ego_planner_node/grid_map/occupancy_inflate' % ID,
                        PointCloud2, queue_size=2)
p_occ = rospy.Publisher('/drone_%d_ego_planner_node/grid_map/occupancy' % ID,
                        PointCloud2, queue_size=2)


def publish_cloud(pub, pts):
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'world'
    pub.publish(pc2.create_cloud_xyz32(h, pts))


print('selftest_fake: 发布假话题 (drone_%d @ (%.2f, %.2f, %.2f), traj_id=94, flag=1)'
      % (ID, X, Y, Z))
print('  注意: 会在 /drone_%d_planning/pos_cmd 上发假指令, 仅限仿真没跑时使用!' % ID)

odom_pt = np.array([X, Y, Z]) + OFF
pillar = [(odom_pt[0] + 0.011, odom_pt[1], odom_pt[2] + dz)
          for dz in np.arange(-0.5, 0.51, 0.1)]
pillar += [(odom_pt[0] + 0.011, odom_pt[1] + dy, odom_pt[2])
           for dy in np.arange(-0.4, 0.41, 0.1)]

rate = rospy.Rate(20)
while not rospy.is_shutdown():
    ms = ModelStates()
    ms.name = ['iris_lidar_%d' % ID]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = X, Y, Z
    ms.pose = [pose]
    ms.twist = [Twist()]
    p_ms.publish(ms)

    od = Odometry()
    od.header.stamp = rospy.Time.now()
    od.header.frame_id = 'world'
    od.pose.pose.position.x = float(odom_pt[0])
    od.pose.pose.position.y = float(odom_pt[1])
    od.pose.pose.position.z = float(odom_pt[2])
    p_od.publish(od)

    c = PositionCommand()
    c.header.stamp = rospy.Time.now()
    c.trajectory_id = 94
    c.trajectory_flag = 1
    c.position.x, c.position.y, c.position.z = X + 0.03, Y + 0.01, Z
    p_cm.publish(c)

    publish_cloud(p_inf, pillar)
    publish_cloud(p_occ, [])
    try:
        rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        break