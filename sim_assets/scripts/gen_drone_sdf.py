#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
为多机编队生成 per-drone 的 Gazebo SDF.

为什么需要它?
------------
PX4 自带的 `iris` 模型 (model://iris) 里, mavlink_interface 插件的
<mavlink_udp_port>/<mavlink_tcp_port> 是**写死**在 iris.sdf 里的 (14560/4560).
单个无人机没问题, 但多机时 PX4 SITL 实例 i 必须用 14560+i / 4560+i,
否则所有实例都去抢同一个 UDP 端口 -> 只有一架能连上 mavros.

PX4 官方靠 `single_vehicle_spawn.launch` 用 jinja_gen.py 渲染
`iris.sdf.jinja` 来做到"每实例不同端口". 本脚本做同样的事, 并额外把
3D LiDAR + IMU (带**每机不同的 ROS 话题名**) 注入到机体里, 这样每架
无人机都能给 FAST-LIO 提供 /drone_<i>/points 与 /drone_<i>/imu.

输出直接打到 stdout, 交给:
    <param command="python3 gen_drone_sdf.py ..." name="model_description"/>
    <node pkg="gazebo_ros" type="spawn_model" args="-sdf -param model_description ..."/>
"""

import argparse
import os
import sys

try:
    import jinja2
except ImportError:
    sys.stderr.write("[gen_drone_sdf] 需要 jinja2 (sudo apt install python3-jinja2)\n")
    raise


# --------------------------------------------------------------------------
#  注入到 iris 机体里的传感器块.
#
#  注意 link 名字故意不叫 imu_link / base_link: PX4 的 iris 已经有
#  base_link 和 imu_link 了, 重名会让 Gazebo 里的关节指向错乱.
#  joint 的 parent 用 base_link (同一 model 内, 直接写相对名字).
#  传感器安装位置与 sim_assets/models/iris_lidar3d/iris_lidar3d.sdf 一致,
#  这样 FAST-LIO 的 extrinsic_T=[0,0,0] 才成立; 而 world 与 camera_init
#  之间的固定平移 = (0.05, 0, 0.20) 是已知常量 (见 vio_offset.py).
# --------------------------------------------------------------------------
SENSOR_BLOCK = """
    <!-- ===== 多机编队专用: 3D LiDAR + IMU (由 gen_drone_sdf.py 注入) ===== -->
    <link name='drone_lidar_link'>
      <pose>0.05 0 0.20 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>1e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-5</iyy><iyz>0</iyz><izz>1e-5</izz>
        </inertia>
      </inertial>
      <visual name='visual'>
        <geometry><cylinder><radius>0.045</radius><length>0.09</length></cylinder></geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/DarkGrey</name>
          </script>
        </material>
      </visual>

      <sensor name='lidar3d' type='gpu_ray'>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>1200</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159265</min_angle>
              <max_angle>3.14159265</max_angle>
            </horizontal>
            <vertical>
              <samples>16</samples>
              <resolution>1</resolution>
              <min_angle>-0.26179939</min_angle>
              <max_angle>0.26179939</max_angle>
            </vertical>
          </scan>
          <range>
            <!-- min = 0.3 m 是**物理盲区**, 不要往小改: 桨尖距雷达仅 0.18 m,
                 机身顶面 0.20 m, 再小就会把自己的桨叶/机身扫进点云
                 (FAST-LIO 跟着漂)。代价: 机体中心距柱面 < ~0.35 m 时该方向无点云,
                 所以规划层必须保证机体不进入这个范围: obstacles_inflation=0.35 的
                 硬边界 0.412 m 正是为此 (> 0.348 机体半径 + 0.05 雷达前置偏移),
                 感知层再配 grid_map 的 min_ray_length 近距保护兜底
                 (见 plan_manage/launch/advanced_param.xml)。 -->
            <min>0.3</min>
            <max>40.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name='gazebo_ros_lidar3d' filename='libgazebo_ros_velodyne_gpu_laser.so'>
          <topicName>{lidar_topic}</topicName>
          <frameName>drone_lidar_link</frameName>
          <organize_cloud>false</organize_cloud>
          <min_range>0.3</min_range>
          <max_range>40.0</max_range>
        </plugin>
      </sensor>
    </link>

    <joint name='drone_lidar_joint' type='fixed'>
      <child>drone_lidar_link</child>
      <parent>base_link</parent>
    </joint>
"""

SENSOR_BLOCK += """
    <link name='drone_imu_link'>
      <!-- 与 drone_lidar_link 原点重合 -> FAST-LIO extrinsic_T=[0,0,0] 精确成立 -->
      <pose>0.05 0 0.20 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-6</iyy><iyz>0</iyz><izz>1e-6</izz>
        </inertia>
      </inertial>
      <sensor name='imu_sensor' type='imu'>
        <always_on>1</always_on>
        <update_rate>200</update_rate>
        <imu>
          <angular_velocity>
            <x><noise type='gaussian'><mean>0.0</mean><stddev>0.0002</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev></noise></x>
            <y><noise type='gaussian'><mean>0.0</mean><stddev>0.0002</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev></noise></y>
            <z><noise type='gaussian'><mean>0.0</mean><stddev>0.0002</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type='gaussian'><mean>0.0</mean><stddev>0.001</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev></noise></x>
            <y><noise type='gaussian'><mean>0.0</mean><stddev>0.001</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev></noise></y>
            <z><noise type='gaussian'><mean>0.0</mean><stddev>0.001</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev></noise></z>
          </linear_acceleration>
        </imu>
        <plugin name='gazebo_ros_imu' filename='libgazebo_ros_imu_sensor.so'>
          <topicName>{imu_topic}</topicName>
          <frameName>drone_imu_link</frameName>
          <!-- 这个插件不读 sensor 层的 update_rate, 只读插件层的 updateRateHZ;
               不写就是 1 Hz, FAST-LIO 需要 ~200 Hz. -->
          <updateRateHZ>200</updateRateHZ>
          <initial_orientation_as_reference>false</initial_orientation_as_reference>
          <gaussianNoise>0.0</gaussianNoise>
        </plugin>
      </sensor>
    </link>

    <joint name='drone_imu_joint' type='fixed'>
      <parent>drone_lidar_link</parent>
      <child>drone_imu_link</child>
    </joint>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--iris-jinja', required=True, help='PX4 的 iris.sdf.jinja 路径')
    ap.add_argument('--sitl-gazebo', required=True,
                    help='PX4 sitl_gazebo-classic 目录 (jinja 的 env_dir)')
    ap.add_argument('--mavlink-id', type=int, default=1)
    ap.add_argument('--mavlink-udp-port', type=int, default=14560)
    ap.add_argument('--mavlink-tcp-port', type=int, default=4560)
    ap.add_argument('--lidar-topic', default='/drone_0/points')
    ap.add_argument('--imu-topic', default='/drone_0/imu')
    ap.add_argument('--output-file', default=None,
                    help='不给就打到 stdout (给 spawn_model -sdf -param 用)')
    args = ap.parse_args()

    env_dir = os.path.abspath(args.sitl_gazebo)
    jinja_path = os.path.abspath(args.iris_jinja)
    if not os.path.isdir(env_dir):
        sys.exit('[gen_drone_sdf] env_dir 不存在: %s' % env_dir)
    if not os.path.isfile(jinja_path):
        sys.exit('[gen_drone_sdf] jinja 模板不存在: %s' % jinja_path)

    env = jinja2.Environment(loader=jinja2.FileSystemLoader(env_dir))
    template = env.get_template(os.path.relpath(jinja_path, env_dir))
    text = template.render({
        'mavlink_id': args.mavlink_id,
        'mavlink_udp_port': args.mavlink_udp_port,
        'mavlink_tcp_port': args.mavlink_tcp_port,
        'serial_enabled': 0,
        'serial_device': '/dev/ttyACM0',
        'serial_baudrate': 921600,
        'hil_mode': 0,
        'cam_component_id': 100,
        'gst_udp_host': '127.0.0.1',
        'gst_udp_port': 5600,
        'external_render_ip': '',
        'video_uri': 'udp://127.0.0.1:5600',
        'mavlink_cam_udp_port': 14530,
        'udp_onboard_gimbal_host_ip': 13030,
        'udp_onboard_gimbal_port_remote': 13030,
        'ros_version': 1,
    })

    # 把传感器块插到最后一个 </model> 之前 (即 iris 机体内部)
    idx = text.rfind('</model>')
    if idx < 0:
        sys.exit('[gen_drone_sdf] 模板里找不到 </model>, 无法注入传感器')
    block = SENSOR_BLOCK.format(lidar_topic=args.lidar_topic,
                                imu_topic=args.imu_topic)
    out = text[:idx] + block + text[idx:]

    if args.output_file:
        with open(args.output_file, 'w') as f:
            f.write(out)
        sys.stderr.write('[gen_drone_sdf] wrote %s\n' % args.output_file)
    else:
        sys.stdout.write(out)


if __name__ == '__main__':
    main()
