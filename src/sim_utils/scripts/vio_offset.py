#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
把 FAST-LIO 输出的 odom / 点云 从 **camera_init(每机各自的起飞点原点)** 平移到公共 world 系.

为什么需要?
----------
FAST-LIO 的 `camera_init` 原点 = 它收到第一帧数据时机体的位置, 也就是"这架无人机的起飞点".
多机编队时每架起飞点不同 (S_i), 于是:
  · 每架的 odom 都是"相对自己起飞点"的, 不共享坐标系;
  · 但 Swarm-Formation 的编队目标是按**同一个世界系**算的
    (goal_i = 编队中心 + relative_pos_i * scale);
  · 机间轨迹交换 (broadcast_traj) 也假设所有人在同一个系里,
    否则避让会算错 (误差正好等于两机起飞点之差).

做法很朴素: odom/点云 各加一个固定平移 (S_i + 传感器安装偏移), 就回到世界系.
平移量由 launch 里的 offset_x/y/z 给出 (见 drone_all.launch).

因为每架无人机的传感器安装位置一样, 这个平移里"传感器安装偏移"的部分对所有机相同,
不会影响机间相对几何; 它只是让 RViz 里显示的位姿和 Gazebo 里的真实位置对得上.

话题/p 参数:
  ~odom_in / ~odom_out    : nav_msgs/Odometry
  ~cloud_in / ~cloud_out  : sensor_msgs/PointCloud2
  ~offset_x/y/z           : 固定平移 (m)
  ~frame_id               : 输出 header.frame_id (默认 world)
"""
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

# PointField.datatype 枚举 -> numpy dtype
_NP_TYPE = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def _pointcloud_dtype(msg):
    """由 PointCloud2.fields 构造 numpy 结构化 dtype (一个点一条记录)."""
    names, formats, offsets = [], [], []
    for f in msg.fields:
        np_t = _NP_TYPE.get(f.datatype)
        if np_t is None:
            return None
        names.append(f.name)
        formats.append(np_t)
        offsets.append(f.offset)
    return np.dtype({'names': names, 'formats': formats,
                     'offsets': offsets, 'itemsize': msg.point_step}), names


class VioOffset(object):
    def __init__(self):
        self.odom_in = rospy.get_param('~odom_in')
        self.odom_out = rospy.get_param('~odom_out')
        self.cloud_in = rospy.get_param('~cloud_in')
        self.cloud_out = rospy.get_param('~cloud_out')
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.offset = np.array([rospy.get_param('~offset_x', 0.0),
                                rospy.get_param('~offset_y', 0.0),
                                rospy.get_param('~offset_z', 0.0)],
                               dtype=np.float32)

        self.odom_pub = rospy.Publisher(self.odom_out, Odometry, queue_size=50)
        self.cloud_pub = rospy.Publisher(self.cloud_out, PointCloud2, queue_size=5)

        rospy.Subscriber(self.odom_in, Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber(self.cloud_in, PointCloud2, self.cloud_cb, queue_size=5)

        rospy.loginfo('[vio_offset] %s -> %s , %s -> %s , offset=(%.3f, %.3f, %.3f)',
                      self.odom_in, self.odom_out, self.cloud_in, self.cloud_out,
                      self.offset[0], self.offset[1], self.offset[2])

    # ---------------------------------------------------------------- odom
    def odom_cb(self, msg):
        m = Odometry()
        m.header = msg.header
        m.header.frame_id = self.frame_id
        m.child_frame_id = msg.child_frame_id
        m.pose.pose = msg.pose.pose
        m.pose.pose.position.x += float(self.offset[0])
        m.pose.pose.position.y += float(self.offset[1])
        m.pose.pose.position.z += float(self.offset[2])
        m.pose.covariance = msg.pose.covariance
        m.twist = msg.twist          # 速度是矢量, 不随平移改变
        self.odom_pub.publish(m)

    # --------------------------------------------------------------- cloud
    def cloud_cb(self, msg):
        built = _pointcloud_dtype(msg)
        if built is None:
            rospy.logwarn_throttle(5.0, '[vio_offset] 点云字段类型不支持, 直通转发')
            m = msg
            m.header.frame_id = self.frame_id
            self.cloud_pub.publish(m)
            return
        dtype, names = built

        arr = np.frombuffer(msg.data, dtype=dtype).copy()
        for i, axis in enumerate(('x', 'y', 'z')):
            if axis in names:
                arr[axis] += self.offset[i]

        m = PointCloud2()
        m.header = msg.header
        m.header.frame_id = self.frame_id
        m.height = msg.height
        m.width = msg.width
        m.fields = msg.fields
        m.is_bigendian = msg.is_bigendian
        m.point_step = msg.point_step
        m.row_step = msg.row_step
        m.is_dense = msg.is_dense
        m.data = arr.tobytes()
        self.cloud_pub.publish(m)


if __name__ == '__main__':
    rospy.init_node('vio_offset')
    VioOffset()
    rospy.spin()
