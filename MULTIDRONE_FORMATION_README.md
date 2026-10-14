# 多机编队仿真: FAST-LIO + Swarm-Formation + Gazebo/PX4 + RViz

在**一个** Gazebo 世界里同时跑 N 架 PX4 SITL 无人机, 每架用 **FAST-LIO** 做状态估计,
所有无人机用 **Swarm-Formation(合并进来的 ego_planner)** 做编队规划与机间避让,
在 **RViz** 里点一次 "2D Nav Goal" 就能让整队按阵型飞过去。

单机那条链路 (`sim_assets/run_sim.sh` + `plan_manage/launch/sim_gazebo.launch`)
**完全没动**, 这份文档讲的是新增的多机链路。

---

## 1. 数据流

```
                    Gazebo 11 (world: formation.world)
   iris_lidar_0        iris_lidar_1        iris_lidar_2     <- 每架一个带 LiDAR+IMU 的 SDF
        |                   |                   |              spawn 在 S0,S1,S2
   /drone_i/points , /drone_i/imu
        v                   v                   v
   fastlio_drone_i (每机一个实例, 只靠 remap 区分话题)
        |  /drone_i/Odometry , /drone_i/cloud_registered      <- 坐标系 = camera_init_i (本机起飞点)
        |
        +--> vision2mavros_drone_i  ->  /drone_i/mavros/vision_pose/pose
        |                              -> PX4 EKF2 的**唯一定位源** (GPS 已关闭)
        v
   vio_offset_drone_i  (+ S_i 常量平移)
        |  /drone_i_visual_slam/odom , /drone_i_pcl_render_node/cloud   <- 坐标系 = 公共 world
        v
   drone_i_ego_planner_node (Swarm-Formation, flight_type=3 编队模式)
        |  /drone_i_planning/pos_cmd            ^
        v                                       | swarm_bridge_i  <==UDP 8081==> 别人的 poly_traj
   drone_i/poscmd2mavros (+ 减掉 S_i)           | /broadcast_traj_from|to_planner
        v
   /drone_i/mavros/setpoint_raw/local  ->  PX4 SITL 实例 i  (OFFBOARD)
```

一架无人机一套 (N=3 时共 33 个节点): `sitl_i` + `iris_lidar_i_spawn` + `mavros` +
`fastlio_drone_i` + `vision2mavros_drone_i` + `vio_offset_drone_i` + `drone_i_ego_planner_node`
+ `drone_i_traj_server` + `drone_i_odom_visualization` + `drone_i/poscmd2mavros` + `drone_i_bridge_node`。

> **PX4 侧定位 = 纯 SLAM**：PX4 SITL 机型 `10015_gazebo-classic_iris` 的默认参数
> 已被改成 `EKF2_GPS_CTRL=0 / EKF2_HGT_REF=3 / EKF2_EV_CTRL=7`，每架 PX4 的
> 位置/高度**完全来自 FAST-LIO**（经 `vision2mavros` 送进 EKF2），GPS 不再参与。
> 因此 `vision2mavros_drone_i` 是**必需**的，删了就无法解锁。改动原因/验证/回退见
> [`PURE_SLAM_LOCALIZATION.md`](PURE_SLAM_LOCALIZATION.md)。

## 2. 怎么跑

```bash
# 终端 1: 起仿真 (默认 3 架; 也支持 1..5)
source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
cd /home/safina/keyan_project/something1017/TECH/sim_assets
./run_formation.sh 3                 # 3 架 + Gazebo GUI
./run_formation.sh 3 --headless      # 无 GUI
./run_formation.sh 3 --no-gen        # 跳过重新生成 per-drone SDF

# 终端 2: RViz
source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
roslaunch sim_utils formation_rviz.launch
```

等 50~70 s (每架 PX4 SITL 启动 + FAST-LIO 初始化 + 自动解锁起飞到 0.5 m),
然后在 RViz 工具栏点 **"2D Nav Goal"**, 在空地上点一个"编队中心"目标点 ——
所有无人机会各自飞到 `relative_pos_<drone_id>` 阵位上, 保持队形一起飞。
到点后原地悬停, 可以再点下一个点 (实测跨 18 m 重新组队, 误差 < 8 cm)。

编队形状/间距在 `src/planner/plan_manage/config/normal_hexagon.yaml`:
* `global_goal/relative_pos_0..6` 每架的相对阵位 (hexagon, 最多 7 架)
* `global_goal/swarm_scale` 缩放 (默认 2.0, 决定阵型大小)
* `optimization/weight_*` 各项权重

## 3. 实测结果 (本机验证过)

3 架, `./run_formation.sh 3`, RViz 点目标点:

| 目标(编队中心) | 期望阵位 | Gazebo 实测 | 误差 |
|---|---|---|---|
| drone 0 | (10.00, 2.00) | (9.999, 1.999, 0.549) | 0.001 m |
| drone 1 | (13.46, 0.00) | (13.491, 0.004, 0.605) | 0.03 m |
| drone 2 | (10.00, −2.00) | (10.009, −2.004, 0.601) | 0.009 m |

第二个目标 (中心 −8,−1, 跨 18 m 重新组队):

| 期望阵位 | Gazebo 实测 | 误差 |
|---|---|---|
| (−8, −1) | (−8.055, −0.957, 0.642) | 0.07 m |
| (−4.54, −3.0) | (−4.571, −2.966, 0.684) | 0.05 m |
| (−8, −5) | (−8.053, −4.941, 0.690) | 0.06 m |

其它验证点:
* FAST-LIO 输出与 Gazebo 真值基本重合 (x/y 差 ~0.03 m), 点云 10 Hz, IMU 200 Hz 配置。
* `/drone_i/points` ~16 Hz(仿真常比设定快)、`/drone_i/Odometry` ~10 Hz、`/drone_i_planning/pos_cmd` 100 Hz。
* 3 架全部 `armed: True` / `mode: OFFBOARD`, 各自起飞到 0.5 m 再交接给规划器。
* 机间通信: `/broadcast_traj_to_planner` 有数据、`/others_odom` ~32 Hz。

## 4. 关键设计点 / 踩过的坑

### 4.1 每架一份 SDF: 端口和话题必须不同 (`gen_drone_sdf.py`)
PX4 的 `iris` 模型里 `mavlink_interface` 的 `<mavlink_udp_port>14560</mavlink_udp_port>` /
`<mavlink_tcp_port>4560</mavlink_tcp_port>` 是**写死**的, 而 LiDAR/IMU 的
`<topicName>` 也是全局名。多机时 N 个实例抢同一个 UDP 端口 -> 只有一架能连上 mavros。
所以用 PX4 自己的 jinja 流程 (`Models/iris/iris.sdf.jinja`) 按实例渲染出端口,
再把 3D LiDAR + IMU (带 `/drone_<i>/…` 话题) 注入机体内部:
`sim_assets/scripts/gen_drone_sdf.py`, 批量生成 `scripts/gen_multi_models.sh`。
* PX4 官方的 `single_vehicle_spawn_sdf.launch` 用了 `xmlstarlet`, 本机**没装**, 所以没走它。
* SDF 生成的是全局名 LiDAR 话题, 所以每架要一份文件; `run_formation.sh` 会先批量生成 5 份。
* 注入的 link 故意叫 `drone_lidar_link` / `drone_imu_link` (PX4 iris 里已经有
  `base_link` 和 `imu_link`, 重名会让关节错乱)。

### 4.2 FAST-LIO 不用改源码, 靠 remap 复用
`laserMapping.cpp` 用的是**全局** NodeHandle, 且发布话题写死为
`/Odometry,/cloud_registered,/cloud_registered_body,/cloud_effected,/Laser_map,/path`。
* 参数 (`common/lid_topic` 等) 必须在 **launch 级** 加载 (不能写在 `<node>` 里, 否则
  它读的是全局名 `/common/lid_topic` 而 `<node>` 里的参数落在私有名字空间 → 读不到)。
* 每机差别只有话题名 → 用 `<remap>` 把输入 `/drone_0/*` 指到本机、把 6 个输出话题
  改成本机的 `/drone_<i>/*`。源码零修改。

### 4.3 坐标系: 每架 FAST-LIO 的原点是自己的起飞点
FAST-LIO 的 `camera_init` 原点 = **本机起飞点**, 所以 N 架的 odom/点云不在同一个系里。
但 Swarm-Formation 的编队目标是按"同一个世界系"算的
(`goal_i = 点击的编队中心 + swarm_scale * relative_pos_i`), 机间轨迹交换也假设同系,
不校正的话机间避让会整体错开 S_i−S_j。
→ `sim_utils/scripts/vio_offset.py` 给每架的 odom **和点云** 各加一个常量平移
`(S_i + 安装偏移)` 回到公共 `world` 系 (点云必须一起平移, 否则 grid_map 里
"机体位置"和"障碍点"不在同一个系)。平移量由 `drone_all.launch` 按起飞点算。

### 4.4 pos_cmd 是世界系, 而 mavros setpoint 是"本机局部系" (必须减 S_i)
`/drone_i_planning/pos_cmd` 是**公共 world 系的绝对坐标**;
`mavros/setpoint_raw/local` 的位置是**以本机 EKF origin(=起飞点)为原点**的局部 ENU。
直接把前者当后者发下去, 第 i 架就会整体平移 S_i:
实测第 1 架 (起飞点 y=2) 目标 y=0 却停在 y≈2.05 —— 正好差 S_1。
→ 给 `to_mavros/poscmd2mavros.cpp` 增加了 `origin_x/y/z` 三个参数 (默认 0),
TRACK 时把 pos_cmd 减去它。**默认 0 时行为与以前完全一致**, 单机/实飞不受影响。
(单机之所以一直没暴露这个问题, 就是因为 S_0=(0,0,0)。)

### 4.5 启动时序 (`launch-prefix` + 延迟)
* `poscmd2mavros` 一上来就会用 PX4 的 `local_position(EKF)` 做
  "解锁+OFFBOARD+起飞到 cur_pos+takeoff_height"。如果它和 PX4/Gazebo 同时启动,
  拿到的是**还没收敛的 EKF 位置**, 实测会把起飞目标算到 3~4 m 高, 飞机自己爬上去,
  z 还超出 `map_size_z` → A* 报 `Ran out of pool` 直接规划失败。
  → `poscmd_start_delay` (默认 55 s) 让它等 PX4+FAST-LIO 都稳了再起。
  这就是单机 `run_sim.sh` 的做法, 这里用 `launch-prefix` 在同一个 launch 里实现。
* FAST-LIO `lio_start_delay` (默认 30 s): 等 Gazebo/IMU 真有数据, 否则重力对齐不准。
* `sim_utils/scripts/delayed_start.sh` 顺手做了 `stdbuf -oL`: rosconsole 的 INFO 走
  stdout, 重定向到文件时是块缓冲, 不加的话日志里只有 WARN/ERROR, INFO 全被吞掉。

### 4.6 `map_size_z` 是 A* 的池子高度
`dyn_a_star.h` 的格子池按 `map_size_x/y/z` 分配, 机体或目标点的 z 一旦超出就
index 越界 → `Ran out of pool, index=366 107 40` → 规划失败。
仿真里给 5.0 m 留足余量 (默认值已从 3.0 改成 5.0, 只在多机链路里)。

### 4.7 catkin_install_python 会把 shebang 改成 conda 的 python
python 节点最初完全不工作 (订阅线程 `ModuleNotFoundError: No module named 'netifaces'`,
话题一个都收不到)。原因不是 env.sh 的 PATH, 而是 `catkin_install_python` 会把脚本第一行
改写成 cmake 配置时的 `PYTHON_EXECUTABLE`; 本机 PATH 里有 conda, 于是生成了
`devel/lib/sim_utils/vio_offset.py -> #!/home/safina/miniconda3/bin/python3`,
而 conda 的 python3.13 里没有 netifaces。
→ 改成 `install(PROGRAMS ...)` 保留源码里写死的 `#!/usr/bin/python3`,
并且 launch 里给该节点加了 `launch-prefix="/usr/bin/python3"` 双保险。

## 5. 新增/修改的文件

新增 (多机链路, 不碰 SF/EP 原有 launch):

| 文件 | 作用 |
|---|---|
| `src/sim_utils/package.xml`, `CMakeLists.txt` | 新 catkin 包 `sim_utils` (纯 python+launch) |
| `src/sim_utils/launch/formation.launch` | 顶层: Gazebo 世界 + N 架 (只写一处起飞点布局) |
| `src/sim_utils/launch/drone_all.launch` | 单架全套: SITL+spawn+mavros+FAST-LIO+平移+规划器+traj_server+odom_vis+poscmd2mavros+bridge |
| `src/sim_utils/launch/formation_rviz.launch` | 多机 RViz |
| `src/sim_utils/rviz/formation.rviz` | 3 架机体/路径/轨迹/编队拓扑显示, Fixed Frame=world |
| `src/sim_utils/scripts/vio_offset.py` | odom+点云 常量平移 (camera_init -> world) |
| `src/sim_utils/scripts/delayed_start.sh` | launch-prefix 用: 延迟启动 + 行缓冲 |
| `sim_assets/scripts/gen_drone_sdf.py` | 生成单架 SDF (jinja 渲染 PX4 iris + 注入 LiDAR/IMU) |
| `sim_assets/scripts/gen_multi_models.sh` | 批量生成 N 架 SDF |
| `sim_assets/worlds/formation.world` | 编队场地: x=0 两道墙留 10 m 门洞 + 4 根柱子 |
| `sim_assets/run_formation.sh` | 一键启动多机仿真 (生成 SDF + 起 launch + 关电池掉电) |
| `MULTIDRONE_FORMATION_README.md` | 本文档 |

修改 (都是向后兼容的):

| 文件 | 改动 |
|---|---|
| `sim_assets/env.sh` | `export SIM_ASSETS_DIR` (launch 里要用 `$(env SIM_ASSETS_DIR)`) |
| `src/TO_MAVROS/src/poscmd2mavros.cpp` | + `origin_x/y/z` 参数, TRACK 里减掉本机起飞点 (默认 0, 单机行为不变) |
| `src/TO_MAVROS/launch/poscmd2mavros.launch` | + 同名 arg/param (默认 0) |

> `src/sim_utils` 通过软链接进 catkin 工作区:
> `TECH/src/ego-planner-swarm/src/sim_utils -> TECH/src/sim_utils` (与 FAST_LIO/TO_MAVROS 一致)。

## 6. 常用调参

`roslaunch sim_utils formation.launch` / `run_formation.sh` 的参数:

| 参数 | 默认 | 说明 |
|---|---|---|
| `num_drones` | 3 | 1..5 (launch 里手写了 5 个块; 要更多就按同样格式加) |
| `start_x` / `start_y` / `spacing` | 0 / 0 / 2.0 | 起飞点一字排开: `y = start_y + spacing*ID` |
| `takeoff_height` | 0.5 | 起飞高度; 注意 SF 会把编队目标的 z 强制成 0.5 |
| `lio_start_delay` | 30 | FAST-LIO 延迟启动 |
| `poscmd_start_delay` | 55 | poscmd2mavros 延迟启动 (等 EKF 收敛) |
| `map_size_x/y/z` | 40/30/5 | 局部地图尺寸 (z 别小于最大飞行高度) |
| `broadcast_ip` | 127.0.0.255 | swarm_bridge 广播地址 (单机跑用 loopback 广播) |
| `max_vel` / `max_acc` | 1.5 / 6.0 | |

编队形状: `plan_manage/config/normal_hexagon.yaml` 的 `relative_pos_*` 与 `swarm_scale`。
换世界: `world` 参数, 或改 `sim_assets/worlds/formation.world`。

## 7. 已知限制 / 注意

* **FAST-LIO 漂移**: LIO 是纯里程计, 各机独立漂移。静止/慢速时激励不足, 漂移会更明显;
  编队本身是按各机的 LIO 系算的, 所以队形看起来是好的, 但**机间相对精度依赖 LIO 漂移**。
  要更高精度可以考虑: 用 PX4 的 `local_position`(公共系、不漂) 当 odom,
  但那样点云也得换成"公共系", 需要额外的外参/对齐 —— 目前没做。
* **N 个 swarm_bridge 在同一台机器上**: 每个 bridge 都会收到所有 UDP 广播(含自己的),
  再各自转发到同一个 `/broadcast_traj_to_planner`, 于是消息被放大约 N 倍
  (实测该话题可以到几百 Hz)。功能正确 (planner 回调里按 `drone_id` 过滤自己 +
  0.25 s 时间戳过滤旧轨迹), 但比较费 CPU。要省资源可以改成一个 bridge 服务所有无人机。
* **GPU 激光雷达**: 模型用的是 `libgazebo_ros_velodyne_gpu_laser.so` (1200×16@10Hz),
  N 架同时开比较吃 GPU。机器吃不住就把 `gen_drone_sdf.py` 里的 sensor 换成
  `type="ray"` + `libgazebo_ros_velodyne_laser.so` (CPU 版) 并把 samples 减半。
* 最高支持 5 架 (受 launch 里手写的块限制); 编队最多 7 架 (受 `relative_pos_0..6` 限制)。
* 单机链路 (`run_sim.sh` / `sim_gazebo.launch`) 没有改动, 仍可照旧用。
