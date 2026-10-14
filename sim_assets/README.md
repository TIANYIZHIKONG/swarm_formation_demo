# PX4 + Gazebo + FAST-LIO + ego-planner-swarm + TO_MAVROS 仿真说明

把 **Gazebo 当"真实世界"**，跑通和真机完全一样的感知→规划→控制链路：

```
        ┌──────────────────────────── Gazebo Classic 11 ────────────────────────────┐
        │  iris 机体(PX4 动力学) + 3D LiDAR(gpu_ray) + IMU   +  障碍物 world        │
        └───────┬───────────────────────────┬──────────────────────────┬────────────┘
   /drone_0/points(PointCloud2)      /drone_0/imu(Imu)            (机体位姿/电机)
                │                           │                          │
                └─────────► FAST-LIO (fast_lio/fastlio_mapping) ◄──────┘
                                     │
                       /Odometry (nav_msgs/Odometry, camera_init)
                       /cloud_registered (PointCloud2, camera_init)
                                     │
                ┌────────────────────┴─────────────────────┐
                │                                          │
     [topic_tools relay]                        to_mavros/vision2mavros  (可选, 默认不开)
   /drone_0_visual_slam/odom                    /mavros/vision_pose/pose
   /drone_0_pcl_render_node/cloud                          │
                │                                     PX4 EKF2 融合(可选, SITL 里默认没必要; 开了偏航会和磁力计冲突, 见 7.2)
                ▼                                          │
        ego-planner (ego_planner/ego_planner_node + traj_server)
                │  /drone_0_planning/pos_cmd (quadrotor_msgs/PositionCommand)
                ▼
        to_mavros/poscmd2mavros  ──►  /mavros/setpoint_raw/local
                │                          │
                └── OFFBOARD 模式 + 解锁 ──► PX4 ──► Gazebo 里的机体运动
```

- **Gazebo**：提供物理环境和传感器（3D 激光 + IMU）。
- **FAST-LIO**：从雷达+IMU 得到里程计（对应真机上的 LIO）。
- **TO_MAVROS**：
  - `vision2mavros`：把 LIO 的 odom 作为外部视觉位姿喂给 PX4 的 EKF2；
  - `poscmd2mavros`（**新增**）：把 ego-planner 的 `PositionCommand` 转成 PX4 的
    offboard 指令，并负责切 OFFBOARD + 解锁 + 降落。
- **ego-planner-swarm**：只用它的 `ego_planner` + `traj_server`，
  **不用**它自带的 `mockamap / pcl_render_node / poscmd_2_odom` 这套假仿真器。
- **RViz**：可视化点云 / 栅格 / 轨迹 / 机体，以及 `roslaunch ego_planner rviz.launch`。

---

## 1. 本次新增/修改的文件

| 路径 | 说明 |
|---|---|
| `TECH/sim_assets/env.sh`（新） | 一次性 source 好 ROS + 工作区 + PX4/Gazebo 的所有环境变量 |
| `TECH/sim_assets/run_sim.sh`（新） | 一键按顺序拉起整条链路 |
| `TECH/sim_assets/models/lidar3d/*`（新） | 3D LiDAR + IMU 的 Gazebo 模型（带 gazebo_ros 插件，发 ROS 话题） |
| `TECH/sim_assets/models/iris_lidar3d/*`（新） | PX4 iris + 上面那套传感器（`include model://iris`） |
| `TECH/sim_assets/worlds/ego_sim.world`（新） | 带柱子/围墙的仿真世界 |
| `TECH/src/FAST_LIO/config/sim_gazebo.yaml`（新） | FAST-LIO 在 Gazebo 下的配置（`lidar_type: 4`） |
| `TECH/src/FAST_LIO/launch/mapping_sim_gazebo.launch`（新） | FAST-LIO 启动文件 |
| `TECH/src/TO_MAVROS/src/poscmd2mavros.cpp`（新） | **关键**：`PositionCommand` → `/mavros/setpoint_raw/local` + OFFBOARD/解锁/起飞/降落状态机。**直接发 ENU，由 mavros 转 NED（见 7.1，别自己再转一遍）** |
| `TECH/src/TO_MAVROS/launch/poscmd2mavros.launch`（新） | 上面那个节点的启动文件 |
| `TECH/src/TO_MAVROS/CMakeLists.txt`、`package.xml`（改） | 增加 `poscmd2mavros` 目标与 `quadrotor_msgs`/`mavros_msgs` 依赖 |
| `TECH/src/ego-planner-swarm/src/planner/plan_manage/launch/sim_gazebo.launch`（新） | 只用 ego_planner + traj_server + topic relay，不启自带仿真器；`flight_type` 已提成 arg（`2`=预设航点，`1`=RViz 手动点，见 6.6） |
| `~/.bashrc`（改） | `slam()` 指向真正的工作区；新增 `sim()` 快捷 source `env.sh` |
| `TECH/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp`（改） | **目标点安全闸门**（`waypointCallback`）：以目标点为球心 0.5 m 球取样判定，四类结果——占据/出地图(**硬拒**)、未观测(默认 WARN 放行)、本地地图盒外(默认 WARN 放行)、视线被已知障碍挡住(**硬拒**)；原先会照样规划"穿墙轨迹"直接撞墙，见 **7.3** |
| `TECH/src/ego-planner-swarm/src/planner/plan_manage/launch/sim_gazebo.launch`（改） | 增加 `odom_visualization` 节点 ⇒ RViz 里能看到**机体 mesh + 飞行轨迹**（之前这两项显示一直是空的） |
| `TECH/src/ego-planner-swarm/src/planner/plan_manage/launch/default.rviz`（改） | 新增 `LiDAR(/cloud_registered)` 点云显示项（墙一眼可见）；默认视角 `ThirdPersonFollower` → **`TopDownOrtho` 正俯视**（手动点目标不再因斜视打偏，见 **7.3**） |
| `TECH/src/ego-planner-swarm/src/planner/plan_manage/launch/advanced_param.xml`、`.../sim_gazebo.launch`（改） | 闸门的 4 个参数/arg：`goal_check_enable`(`true`)、`goal_check_radius`(`0.5`)、`goal_check_strict`(**`false`**)、`goal_check_los`(`true`)，都可从 `roslaunch ... goal_check_strict:=true` 覆盖（见 **7.3**） |


> `env.sh` 有个必须保留的细节：文件**末尾**的 `eval "$_ENV_SH_SAVED_OPTS"` 用来恢复调用者
> 原来的 shell 选项（因为中间要临时 `set +u` 才能安全 source PX4 的 `setup_gazebo.bash`）。
> 往 `env.sh` 里追加内容时，**不要加到这段之后**（详见 7.2 坑表里"什么都没发生"那条）。

---

## 2. 前置: 把 4 个仓库放进同一个 catkin 工作区

现状:

```
TECH/src/ego-planner-swarm/{src,build,devel}   <- 目前实际在用的 catkin 工作区
TECH/src/FAST_LIO, TECH/src/TO_MAVROS          <- 在工作区外面，编译不进去
```

`to_mavros` 现在依赖 `quadrotor_msgs`，`fast_lio` 也要一起编，所以用 **软链接** 挂进工作区
（不移动文件、不影响 git 仓库结构；`build/`、`devel/` 已在 `.gitignore` 里）：

```bash
WS=/home/safina/keyan_project/something1017/TECH/src/ego-planner-swarm
SRC=/home/safina/keyan_project/something1017/TECH/src

ln -sfn $SRC/FAST_LIO  $WS/src/FAST_LIO
ln -sfn $SRC/TO_MAVROS $WS/src/TO_MAVROS
ls -l $WS/src/
```

`FAST_LIO/CMakeLists.txt:45` 把 `livox_ros_driver2` 列为 **必需** 组件，所以驱动也必须进工作区：

```bash
ln -sfn $SRC/livox_ros_driver2 $WS/src/livox_ros_driver2
ls -l $WS/src/     # 应有 FAST_LIO / livox_ros_driver2 / TO_MAVROS 三个软链接
```

再编译（`-j1` 是官方 README 建议，避免内存爆掉；机器内存够可以 `-j8`）：

```bash
cd $WS
catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release -j8
source devel/setup.bash
rospack find fast_lio livox_ros_driver2 to_mavros   # 三个都要能找到
ls devel/lib/fast_lio/ devel/lib/livox_ros_driver2/ devel/lib/to_mavros/
```

关于 `livox_ros_driver2`（踩过的坑）：

- **必须带 `-DROS_EDITION=ROS1`**。它 `/CMakeLists.txt` 第 2 行用 `if(ROS_EDITION STREQUAL "ROS1")`
  区分 ROS1(catkin) / ROS2(ament) 两套完全不同的构建逻辑，不传就会走 ament 分支然后
  `find_package(ament_cmake_auto)` 失败。**现已在该文件开头加了
  `if(NOT DEFINED ROS_EDITION) set(ROS_EDITION "ROS1") endif()` 兜底**，
  所以以后不传也能编；但若将来 `git pull` 这个仓库，注意这 4 行可能有冲突。
- 它 `CMakeLists.txt:94` 写死了 `find_library(... liblivox_lidar_sdk_static.a /usr/local/lib)`，
  即**只认 `/usr/local` 里装的 Livox-SDK2**，不用你自己 `build/` 目录里的那份。
  编之前先确认 `sudo make install` 过；`file /usr/local/lib/liblivox_lidar_sdk_shared.so`
  应该是 `x86-64`（若是 `ARM aarch64` 就是从前一台机器拷过来的，必须重装）。
- `package.xml` 必须是 **package_ROS1.xml 那份**（含 `<buildtool_depend>catkin</buildtool_depend>`）。
  若曾被 `build.sh ROS2` 换成 ROS2 版，用 `cp package_ROS1.xml package.xml` 换回来，
  否则 catkin 会因为找不到 `catkin` 构建类型而报错。
- **不要直接跑 `livox_ros_driver2/build.sh`**：它假设 `ws/src/livox_ros_driver2` 的目录布局，
  会 `rm -rf ../../build ../../devel ../../install`、`rm -f ../CMakeLists.txt`，
  然后在 `TECH/`（而不是 `ego-planner-swarm/`）下新建一整套工作区，
  并把 `TECH/src` 下所有包再编一遍，产生两套互相打架的 devel 空间。

> 说明：之前 `~/.bashrc` 里的 `slam()` 是指向 `TECH/setup.bash` 的，但那个文件并不存在，
> 所以它一直没生效。现在已改为 `source .../ego-planner-swarm/devel/setup.bash`。
> 也可以直接用新增的 `sim()`。

---

## 3. 编译 PX4 SITL + Gazebo Classic 插件

**当前状态：`PX4-Autopilot/build/px4_sitl_default/bin/` 里只有 `px4-alias.sh`，
没有 `px4` 可执行文件 → PX4 还没真正编译过。** 必须先做这一步。

```bash
cd ~/PX4-Autopilot
make px4_sitl_default sitl_gazebo-classic        # 首次约 15~40 分钟
```

这一步会：

1. 编出 `build/px4_sitl_default/bin/px4`；
2. 编出 Gazebo Classic 插件到 `build/px4_sitl_default/build_gazebo-classic/*.so`
   （`libgazebo_mavlink_interface.so`、`libgazebo_imu_plugin.so` …）；
3. 把 `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/*.sdf.jinja`
   渲染成 `models/*.sdf`（**写在源码树里**）。`model://iris` 就是靠这一步才存在的，
   所以在本 README 的自定义模型里 `include model://iris` 的前提是先跑完这个 make。

验证：

```bash
ls -l   ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
ls      ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf
ls      ~/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic/*.so | head
```

> 如果想连 Gazebo 模型一起重新生成：
> `rm -f ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/*/*.sdf`
> 再重新 `make px4_sitl_default sitl_gazebo-classic`。

---

## 4. 环境准备（每个新终端）

```bash
source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
# 等价于(旧写法) source ~/.bashrc 之后敲 `sim`
```

它做了这些事：

- `source /opt/ros/noetic/setup.bash`
- `source <ws>/devel/setup.bash`（ego_planner / quadrotor_msgs / fast_lio / to_mavros）
- `source $PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash`
  → 设置 `GAZEBO_PLUGIN_PATH` / `GAZEBO_MODEL_PATH` / `LD_LIBRARY_PATH`
- 追加本仓库的 `sim_assets/models`、`sim_assets/worlds`
- 追加 `ROS_PACKAGE_PATH`，使 `$(find px4)`、`$(find mavlink_sitl_gazebo)` 可用
- `ROS_HOSTNAME=localhost`、`ROS_MASTER_URI=http://localhost:11311/`

自检：

```bash
rospack find px4                            # /home/safina/PX4-Autopilot
rospack find mavlink_sitl_gazebo            # .../sitl_gazebo-classic
echo $GAZEBO_MODEL_PATH | tr ':' '\n' | head -3
```

---

## 5. 启动

### 5.1 一键（推荐先跑这个）

```bash
cd /home/safina/keyan_project/something1017/TECH/sim_assets
./run_sim.sh              # 默认: 按 sim_gazebo.launch 里的预设航点自动依次飞
./run_sim.sh --manual     # 不飞预设航点, 改为在 RViz 里用 "2D Nav Goal" 手动点目标点
                          # (等价于给 ego_planner 传 flight_type:=1, 用法见 6.6)
./run_sim.sh --headless   # 不开 Gazebo 界面(远程/无显示器)
```

按顺序启动并各自写日志到 `sim_assets/logs/`：

| 顺序 | 进程 | 日志 |
|---|---|---|
| 1 | `roslaunch px4 mavros_posix_sitl.launch`（PX4 SITL + Gazebo + mavros，等 30s；已带 `interactive:=false`） | `px4_gazebo.log` |
| 1.5 | `param set SIM_BAT_MIN_PCT 100`（关掉 SITL 模拟电池掉电，否则解锁 10s 后被自动上锁） | — |
| 2 | `roslaunch fast_lio mapping_sim_gazebo.launch` | `fast_lio.log` |
| 3 | **（可选）** `roslaunch to_mavros vision2mavros.launch`，默认**不启**，加 `--vision` 才启 | `vision2mavros.log` |
| 4 | `roslaunch ego_planner sim_gazebo.launch`（默认 `flight_type:=2` 飞预设航点；`--manual` 时传 `flight_type:=1`，见 6.6） | `ego_planner.log` |
| 5 | `roslaunch to_mavros poscmd2mavros.launch`（切 OFFBOARD + 解锁 + 跟踪；想先悬停到 1m 再加 `takeoff_enable:=true`） | `poscmd2mavros.log` |

再开一个终端看 RViz：

```bash
source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
roslaunch ego_planner rviz.launch
```

> RViz 的 Fixed Frame 用 `world`（launch 里已加 `world -> camera_init` 的静态 TF）。

### 5.2 手动分步（排查问题建议用这个，5 个终端）

```bash
# 终端1: PX4 + Gazebo + mavros
source .../sim_assets/env.sh
roslaunch px4 mavros_posix_sitl.launch \
    vehicle:=iris \
    sdf:=$HOME/keyan_project/something1017/TECH/sim_assets/models/iris_lidar3d/iris_lidar3d.sdf \
    world:=$HOME/keyan_project/something1017/TECH/sim_assets/worlds/ego_sim.world \
    gui:=true interactive:=false
#  interactive:=false 必须加! 否则 PX4 交互 shell 在后台会把日志刷到 GB 级
#  另开一处执行(关掉模拟电池掉电, 否则解锁 10s 后会被自动上锁):
#    rosservice call /mavros/param/set "{param_id: 'SIM_BAT_MIN_PCT', value: {integer: 0, real: 100.0}}"
#  如果想用 PX4 自带的仓库场景(更密集): world:=$(find mavlink_sitl_gazebo)/worlds/warehouse.world

# 终端2: FAST-LIO
roslaunch fast_lio mapping_sim_gazebo.launch

# 终端3(可选, 默认不用): LIO odom -> mavros vision_pose
#   在 SITL 里开了它 PX4 会因为偏航参考冲突报 "Preflight Fail: Yaw estimate error",
#   详见第 7.2 / 第 8 节。只想跑通链路的话跳过这步。
roslaunch to_mavros vision2mavros.launch

# 终端4: ego-planner
roslaunch ego_planner sim_gazebo.launch

# 终端5: pos_cmd -> mavros offboard
roslaunch to_mavros poscmd2mavros.launch

# 终端6(可选): RViz
roslaunch ego_planner rviz.launch
```

RViz 2D Nav Goal 手动选点（`flight_type:=1`）：

```bash
roslaunch ego_planner sim_gazebo.launch flight_type:=1
```

---

## 6. 分步验证（**强烈建议按这个顺序确认每一环**）

### 6.1 PX4 + Gazebo + mavros

```bash
rostopic echo -n1 /mavros/state          # connected: True
rostopic hz /clock                        # ~real-time 更新(sim time)
rostopic hz /drone_0/imu                  # ~125 Hz (见下方说明)
rostopic hz /drone_0/points               # ~10 Hz
rostopic echo -n1 /drone_0/points/width   # 14600 左右一帧
rviz 里 Add -> PointCloud2 -> /drone_0/points  (Fixed Frame: lidar3d::lidar_link)
```

> IMU 为什么是 ~125 Hz 而不是 200：`gazebo_ros_imu_sensor` 的 `<updateRateHZ>` 设了 200，
> 但 world 的物理步长是 4 ms（250 Hz），传感器只能在物理步边界触发，实际落在 250/2=125 Hz。
> 125 Hz 对 FAST-LIO 完全够用；想精确 200 Hz 可以把 world 的 `real_time_update_rate` 提到 400。

### 6.2 FAST-LIO

```bash
rostopic hz /Odometry                     # 应与 LiDAR 同频(10 Hz 左右)
rostopic echo -n1 /Odometry/pose/pose     # 把无人机拿起来/推一下, 位姿应该跟着变
rostopic hz /cloud_registered             # 配准后的点云
rosrun tf tf_echo camera_init body
```

### 6.3 TO_MAVROS

```bash
rostopic hz /mavros/vision_pose/pose      # ~30 Hz
rostopic echo -n1 /mavros/vision_pose/pose
# 此时在 RViz 里把 /mavros/local_position/pose(GPS/气压得到的) 和
# /mavros/vision_pose/pose(LIO 得到的) 画在一起, 两条轨迹应该基本重合
```

### 6.4 ego-planner

```bash
rostopic hz /drone_0_planning/bspline     # 有轨迹说明规划在跑
rostopic hz /drone_0_planning/pos_cmd     # ~100 Hz
rostopic echo -n1 /drone_0_planning/pos_cmd
rostopic hz /drone_0_pcl_render_node/cloud  # relay 出来的点云
```
RViz 里能看到 `/drone_0_planning/pos_cmd`、`/drone_0_grid/grid_map/occupancy_inflate`
之类的栅格和轨迹，说明规划已经正常。

### 6.5 poscmd2mavros（真正飞起来）

```bash
rostopic echo /mavros/state                # mode 变成 OFFBOARD, armed: True
rostopic hz   /mavros/setpoint_raw/local   # ~50 Hz
```

**实测通过的结果**（本机 2026-10-10，iris + ego_sim.world）：

```
t=+5s   x=0.00   z=0.003   armed:False  OFFBOARD      # 刚起 poscmd2mavros
t=+10s  x=-0.17  z=0.414   armed:True   OFFBOARD      # 解锁成功, 起飞
t=+15s  x=-0.01  z=0.541   armed:True   OFFBOARD      # 爬到 1 m 悬停
--- 起 ego-planner ---
t=+10s  x=4.36   z=0.81    armed:True   OFFBOARD      # 向东穿过 x=5 门洞, 冲 (8,0,1)
t=+20s  x=-7.55  z=0.97    armed:True   OFFBOARD      # 折返向西, 连穿两道门洞
t=+30s  x=-7.98  z=0.99    armed:True   OFFBOARD
t=+60s  x=-7.95  z=1.05    armed:True   OFFBOARD      # 稳定悬停在 (-8,0,1)
```

`poscmd2mavros.log` 里会依次打印
`OFFBOARD requested -> arming requested -> OFFBOARD+armed -> TAKEOFF/TRACK`。

> ⚠️ 上面这份数据是**时序刚好**的一次。预设模式有"第一段航点被空跑"的竞态
> （ego-planner 比 `poscmd2mavros` 早 5 s 起，轨迹时间轴在飞机还没解锁时就开始走，
> 到点判断用的是轨迹自身进度而不是实际位置）。表现就是：解锁起飞后**直接飞第二段
> `(-8,0,1)`**、最后停在那儿。原因与规避见 **7.2** 坑表；想稳定可复现建议直接用
> **6.6 的 `--manual`**。

---

### 6.6 用 RViz 手动下发目标点（`flight_type:=1`）

前面 6.5 跑的是 launch 里**写死的预设航点**（`flight_type:=2`，无人值守）。
想让飞机听你现场指挥，用 `--manual` 换成 **RViz 的 2D Nav Goal** 模式：

```bash
./run_sim.sh --manual          # 等价于 roslaunch ego_planner sim_gazebo.launch flight_type:=1
```

等它起飞悬停后，另开终端起 RViz：

```bash
source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
roslaunch ego_planner rviz.launch
```

在 RViz 工具栏点 **`2D Nav Goal`**，在地面上拖一个箭头即可（箭头的方向/yaw 不生效，只用 x/y）。

**为什么这样就能用**（代码依据，逐条对得上）：

| 项 | 值 / 位置 |
|---|---|
| planner 在哪收目标 | `ego_replan_fsm.cpp:64`：`flight_type==1` 时订阅 `/move_base_simple/goal` |
| RViz 往哪发 | `launch/default.rviz` 里已带 `rviz/SetGoal` 工具，`Topic: /move_base_simple/goal` → 天然对齐，不用改 |
| 坐标系 | `default.rviz` 的 `Fixed Frame: world`；`sim_gazebo.launch` 里加了单位静态 TF `world -> camera_init`，而 FAST-LIO 的 odom 就在 `camera_init`（起点即原点）⇒ **RViz 里点的坐标 == planner 的坐标** |
| 高度 z | RViz 地面工具给的是 z=0，但 `ego_replan_fsm.cpp:216` 把它强制成 `end_wp=(x, y, 1.0)` ⇒ **只飞 1 m 高，不会撞地**（另有 `:213` 的 `z < -0.1 直接 return` 兜底） |
| 目标点合法性 | `ego_replan_fsm.cpp:345` 的 `waypointCallback()` 先调 `checkGoalSafety()`（`:266`，以目标点为心、半径 `goal_check_radius`=0.5 m **球形取样**）：**占据/出地图**(`1`) 与 **视线被已知障碍挡住**(`4`) ⇒ **硬拒绝** + RViz **红球**，不下发（`Triggered!` 不增加）；**未观测**(`2`) / **本地地图盒外**(`3`) ⇒ 默认 WARN 后**放行**，`goal_check_strict:=true` 时也拒绝。详见 **7.3** |
| 到点之后 | 状态回 `WAIT_TARGET`，`traj_server` 保持末点 ⇒ **原地悬停**，可以继续点下一个点 |
| 需不需要 trigger | 不需要。`have_trigger_` 只在 `PRESET_TARGET` 分支被订阅的 `/traj_start_trigger` 使用；manual 分支靠 `planNextWaypoint()` 里 `exec_state_==WAIT_TARGET → GEN_NEW_TRAJ` 直接跳（`:183`） |

**实测（2026-10-10，`--manual`，连发两个目标点）**：

```
t= 4.0  x=-0.28 y=-0.39 z=0.53  OFFBOARD armed=True   # 起飞后先悬停
########## GOAL 目标1 (6.0, 0.0) ##########
t= 6.0  x= 1.24 z=0.66                                 # 开始向东
t= 8.0  x= 4.47 z=0.94                                 # 穿过 x=5 门洞
t=12.0  x= 6.03 z=1.02                                 # 到达
t=14~30 x≈5.93~6.05  y≈±0.1  z≈0.98~1.07               # ✅ 稳定悬停 (6,0,1) 约 20 s
########## GOAL 目标2 (-6.0, 0.0) ##########
t=32.0  x= 4.54 z=0.89                                 # ✅ 立刻折返
t=38.0  x=-4.06 z=0.98                                 # 穿过西侧门洞
t=44~58 x≈-5.97~-6.08  z≈0.96~1.04                     # ✅ 稳定悬停 (-6,0,1) 约 15 s
```

全程 `mode=OFFBOARD armed=True`，**没有掉出 OFFBOARD、没有自动上锁**。
`ego_planner.log` 里的 FSM 轨迹证明"到点 → 等新目标 → 再规划"是闭环的：

```
Triggered!                    <- 收到目标1
GEN_NEW_TRAJ -> EXEC_TRAJ
EXEC_TRAJ <-> REPLAN_TRAJ  (多次, 边飞边重规划)
EXEC_TRAJ -> WAIT_TARGET      <- 到达目标1, 悬停
Triggered!                    <- 收到目标2 (证明到点后仍能接收新目标)
GEN_NEW_TRAJ -> EXEC_TRAJ
EXEC_TRAJ <-> REPLAN_TRAJ  (多次)
EXEC_TRAJ -> WAIT_TARGET      <- 到达目标2, 悬停
```

**不想开 RViz 时怎么发**（远程/headless 方便；坐标是 world 系 == planner 系）：

```bash
source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 6.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

**能点哪些地方**（对应 `ego_sim.world` 布局，见该文件顶部示意图）：
- x=±5 各有一道墙，**只在 y∈[−1.5, 1.5] 留 3 m 门洞** ⇒ 要穿墙必须走门洞；
- 柱子（半径 0.5）在 `(±8,±4)`、`(0,±6)`、`(0,9)`，别点它们身上；
- 最省事就是**沿 y=0 点**：`(6,0)`、`(-6,0)`、`(3,0)`、`(-3,0)`、`(8,0)`、`(-8,0)`。

**注意（重要）**：
- **别快速连点**。`ego_replan_fsm.cpp:187` 在"飞行途中收到新目标"时会**在回调里忙等**
  直到状态回到 `EXEC_TRAJ`；连点、或在 `EMERGENCY_STOP` 时点，有把规划节点卡住的风险。
  要改目标，等飞机停下（悬停）再点最稳。
- 目标点**绝对不要点在墙体/柱子内部**：实测会让无人机**直接撞墙坠机**
  （2026-10-10 复现全过程见 **7.3**）。现在 `waypointCallback` 里有安全闸门会拦住这类点。
- **要点墙后面的点，先飞到"看得见它"的位置**：闸门会拒绝"无人机→目标点直线穿过已知墙/柱"的点
  （局部规划器在这种目标上会规划出穿墙轨迹）。例如要飞东侧 `(7, 2.5)`，先点门洞前的 `(4, 0)`，
  再点 `(7, 2.5)` —— 直线正好从门洞过 ⇒ 放行并安全飞过去（**7.3** 表 T4/T5 实测）。
- 拦不住"刚好点在障碍物边缘外一点点"的情况，所以还是建议看清再点：
  视角切到 **`TopDownOrtho` 正俯视** + 看橙色 LiDAR 点云里的墙。
- 飞行高度没法在 launch 里调（代码写死 1.0 m）。想改高度得改
  `ego_replan_fsm.cpp:216` 那行。

---

## 7. 常见坑

### 7.1 mavros `setpoint_raw/local` **会自己做 ENU→NED**（最容易踩，已修）

`poscmd2mavros` 里**绝对不要再手动做 ENU→NED**。实测（往 `/mavros/setpoint_raw/local`
发特征值，再读 PX4 的 uORB `trajectory_setpoint`）：

| 发进去 (frame_id 填 `map` 或留空都一样) | PX4 实际收到 |
|---|---|
| pos=(1, 2, 3), yaw=0.70  | pos=**(2, 1, −3)**, yaw=**0.871** = π/2−0.70 |
| pos=(4, 5, 6), yaw=1.10  | pos=**(5, 4, −6)**, yaw=**0.471** = π/2−1.10 |
| pos=(−7, −8, −9), yaw=−0.50 | pos=**(−8, −7, 9)**, yaw=**2.071** = π/2+0.50 |
| vel=(0.4, 0.5, 0.6), yaw_rate=+0.9 | vel=**(0.5, 0.4, −0.6)**, yawspeed=**−0.9** |

即 **`NED_x = ENU_y`、`NED_y = ENU_x`、`NED_z = −ENU_z`、`yaw_ned = π/2 − yaw_enu`、
`yaw_rate` 取反**，而且**无论 `frame_id` / `coordinate_frame` 怎么填都会转**。

所以正确做法是：**直接发 ENU 值**，让 mavros 去转。
如果自己也转一遍（就是最初的写法），就会**双重转换**：x/y 互换 + z 反号。
后果非常隐蔽——**能解锁、能进 OFFBOARD、日志毫无报错，但飞机就是不动**：
PX4 收到的目标是"往地下 2 m"，一直判定 `landed=true`，只给 `actuator_motors ≈ 0.002`
（0.2%）的推力，10 秒后还被 `COM_DISARM_PRFLT` 自动上锁（提示
`Disarmed by auto preflight disarming`），看起来像"飞机坏了"。
排查方法：用 `pyulog` 读 PX4 的 `.ulg`（在 `~/.ros/log/<日期>/`），看
`trajectory_setpoint` 的 `position` 和你发的是不是一致。

### 7.2 其它坑

| 现象 | 原因 / 处理 |
|---|---|
| `spawn_model` 抛 `ModuleNotFoundError: No module named 'netifaces'`，Gazebo 里**根本没有无人机**，`/drone_0/*` 一个话题都没有 | `gazebo_ros/spawn_model` 的 shebang 是 `#!/usr/bin/env python3`，PATH 里 miniconda 在前 → 用 python3.13 跑 ROS 的 3.8 代码。`env.sh` 已自动把 `/usr/bin` 提到 PATH 最前 |
| PX4 日志几分钟涨到 GB 级，里面全是 `pxh>` | `mavros_posix_sitl.launch` 默认 `interactive:=true` 会起 PX4 交互 shell；nohup/后台时 stdin 是 EOF，shell 死循环打印提示符。**必须加 `interactive:=false`**（`run_sim.sh` 已加） |
| `/drone_0/imu` 只有 **1 Hz** | `gazebo_ros_imu_sensor` **不读 sensor 层的 `<update_rate>`**，只读插件层的 `<updateRateHZ>`（缺省 1 Hz）。已在 `lidar3d.sdf` 里加 `<updateRateHZ>200</updateRateHZ>` |
| 能解锁，但 10 秒后被自动上锁（`Disarmed by auto preflight disarming`），飞机永远不起飞 | 两种可能：①坐标双重转换（见 7.1）；②SITL 模拟电池：解锁期间电池按 `SIM_BAT_DRAIN`(默认 60 s) 从 100% 掉到 `SIM_BAT_MIN_PCT`(50%)，电压跌破门槛 → `Preflight Fail: Battery unhealthy`。`run_sim.sh` 已在解锁前 `param set SIM_BAT_MIN_PCT 100` 关掉掉电 |
| OFFBOARD 时 PX4 报 `Preflight Fail: Yaw estimate error` 拒绝解锁 | SITL 里 PX4 有 GPS+磁力计，再把 LIO 位姿喂给 `/mavros/vision_pose` 会让偏航参考冲突。**默认不要开 `vision2mavros`**（`run_sim.sh --vision` 才开），要用见第 8 节 |
| 航点永远到不了，还会撞墙 | 航点必须落在 world 的空旷处。`ego_sim.world` 是"x=±5 两道墙 + y∈[−1.5,1.5] 门洞"，对应航点 `(8,0,1)→(−8,0,1)→(0,0,1)`；换 world 记得同步改 `sim_gazebo.launch` 的 `point*` |
| world 加载失败 `Comment must not contain '--'` | SDF/XML 注释里不能出现连续两个减号（画 ASCII 图时容易犯） |
| `$(find mavlink_sitl_gazebo)` / `$(find px4)` 失败 | 没有 `source env.sh`，`ROS_PACKAGE_PATH` 里缺 PX4 的两个目录 |
| 预设航点模式**第一段航点被"空跑"掉**（飞机一解锁起飞就直接飞第二段 `(-8,0,1)`，甚至一直停在那而不去 `(0,0,1)`） | 时序竞态，与代码/航点配置无关：`run_sim.sh` 里 ego-planner 比 `poscmd2mavros` 早 5 s 起，FAST-LIO 一出 odom 它就 `readGivenWps()`→ 立刻开始跑第一段轨迹的**时间轴**，而飞机此时还没解锁/起飞。FSM 判断"到没到航点"用的是**规划轨迹自身的进度**（`ego_replan_fsm.cpp:553,556`，不是实际 odom 位置）⇒ 第一段被空跑。**用 `--manual` 完全没有这个问题**（手动模式没目标点就没有轨迹，时间轴不会空转）；要保留预设模式就把 `run_sim.sh` 里 planner 与 poscmd2mavros 之间那 5 s 调大，或等飞机悬停后再单独起 planner |
| **`./run_sim.sh` 跑下去"什么都没发生"**（日志空的、`logs/` 里没东西、进程也没有） | PX4 的 `setup_gazebo.bash:19` 写的是 `export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:...`，而 **ROS 并不设置 `GAZEBO_PLUGIN_PATH`**。脚本里的 `set -u` 让这个"未定义变量"**直接终止整个脚本**，错误又被重定向吞掉 ⇒ 静默退出。**已修**：`env.sh` 现在会在内部临时 `set +u`、结束前用 `set +o` 快照恢复调用者的选项（顺手给三个 Gazebo 变量补了默认值）。**改 `env.sh` 后请务必保留文件末尾的恢复逻辑** |
| Gazebo 里模型加载不出来 / `model://iris` 找不到 | 先跑 `make px4_sitl_default sitl_gazebo-classic` 生成模型和插件 |
| `make px4_sitl_default` 在 miniconda 的 python 下报错 | PX4 v1.14 的工具链要在 python3.8 上跑：`PATH=/usr/bin:$PATH make px4_sitl_default CMAKE_ARGS="-DPYTHON_EXECUTABLE=/usr/bin/python3"`，并 `unset PYTHONPATH` |
| `RLException: unable to contact ROS master` | 旧 shell 里 `ROS_HOSTNAME` 是别的机器名；`export ROS_HOSTNAME=localhost`，或重开终端 |
| 节点起来后"不动"、定时器不触发 | `use_sim_time` 生效但 `/clock` 还没发布（Gazebo 没起）。必须先起 Gazebo，再起其它节点 |
| PX4 拒绝 OFFBOARD | 必须先以 >2 Hz 连续发 setpoint 再切模式（`poscmd2mavros` 已 50 Hz 在发） |
| FAST-LIO 漂移/散架 | 检查 `sim_gazebo.yaml` 的 `lid_topic`/`imu_topic`；确认 `lidar_type: 4`；`blind` 调大一点滤掉打在自己身上的点 |
| FAST-LIO 刷 `No point, skip this scan!` | 只在**启动最初几帧**出现是正常的（`IMU_init` 阶段还没产出点云，`ImuProcess::Process` 直接 return）。如果一直刷才是问题 |
| CPU 跑不动 | 把 `lidar3d.sdf` 的 `<samples>` 从 1200 降到 600；或把 `launch/mapping_sim_gazebo.launch` 里 `point_filter_num` 调大 |
| GPU ray 起不来（无显卡/纯 headless） | 把 `lidar3d.sdf` 里 `type="gpu_ray"` 改成 `type="ray"`，插件换 `libgazebo_ros_velodyne_laser.so`。**本机有 RTX 4060 + 真 X 显示（`DISPLAY=:1`），`gpu_ray` 实测可用** |

### 7.3 "RViz 里点的坐标不对、无人机撞墙" —— 真实原因与修复 + **目标点安全闸门**（2026-10-10 实测）

**症状**：`./run_sim.sh --manual` 起飞悬停后，在 RViz 用 `2D Nav Goal` 点一个"看着像空地"的位置，
无人机一头扎进墙里 / 掉到地上，看起来像"RViz 下发的坐标和 planner 的坐标没同步"。

**实测结论：坐标系完全是对的，问题在别处。** 无人机悬停时同时读三套坐标：

```
GAZEBO 真值 iris   x=1.38  y=1.65  z=1.10      <- /gazebo/model_states
MAVROS map(ENU)   x=1.37  y=1.61  z=1.09      <- /mavros/local_position/pose
LIO camera_init   x=1.35  y=1.64  z=1.03      <- /drone_0_visual_slam/odom
TF  world -> camera_init : 平移(0,0,0) 四元数(0,0,0,1)     <- 单位变换
```

再把 planner 实际吃到的点云（`/drone_0_pcl_render_node/cloud`，`frame_id=camera_init`）
按 x 做直方图：

```
x= 4.0.. 5.0  n=484     <- 东墙
x=-5.0..-4.0  n=506     <- 西墙      (.world 文件里墙就在 x=±5, 厚 0.4)
```

⇒ `camera_init == world == mavros ENU`，点云里的墙也和 `ego_sim.world` 一致，
**RViz 点的 (x,y) 与 planner 用的 (x,y) 完全同一个系**。所以撞墙不是坐标串了，而是下面两条：

**原因 ①：planner 不检查目标点是否可通行。** 目标点落在墙体内部时，
`planNextWaypoint()` 照样规划出一条"穿墙"轨迹，无人机直接撞上去。
复现（目标点故意打在东墙本体 `(5.0, 3.0)`，墙是 x∈[4.8,5.2]、y∈[1.5,8]）：

```
t= 0  x= 6.91 y=2.44 z=1.11  roll=-0.1 pitch= 0.1   <- 原本平稳悬停
t= 1  x= 6.71 y=2.57 z=1.79  roll=-14.5 pitch=-2.3  <- 一头扎向墙
t= 2  x= 5.45 y=2.98 z=1.02             pitch=-19.4 <- 撞墙
t= 3  x= 5.46 y=2.81 z=0.07                         <- 掉到地上 = 坠机
t= 4~22  在原地翻滚搅动(z 在 0.1~1.4 跳, pitch 20~30°)
t=29~     x= 8.60 y=1.54 z=0.05  一动不动           <- 瘫在地上(再也起不来)
```

**已修（2026-10-10 二次加固后的最终实现）**：`waypointCallback()`（`ego_replan_fsm.cpp:345`）
现在先调 `checkGoalSafety()`（`:266`）——以目标点为球心、`goal_check_radius`（默认 **0.5 m**）
为半径，**按地图分辨率球形取样**，逐格用 `getInflateOccupancy()` / `getOccupancy()` 判定，
返回 0~4 五类结果：

| 返回值 | 含义（判定依据） | 处理 |
|---|---|---|
| `0` | 球内**全部格都被观测且空闲** | `cout << "Triggered!"`，正常下发（**本仿真基本不会出现**，原因见下） |
| `1` | 球内有**占据格**（墙/柱/膨胀层）或**出地图**（`:311`） | **硬拒绝**：`ROS_WARN("[EGO] Goal (x, y) REJECTED: N/M cells around it are occupied or outside the map ...")` + RViz **红球**（`/drone_0_ego_planner_node/goal_point`，id=0） |
| `2` | 球内有**未观测(UNKNOWN)格**（`:335`） | 默认**放行** + WARN（`... ACCEPTED but NOT VERIFIED: N/M cells around it are UNKNOWN ... but the line of sight to it is clear ...`）；`goal_check_strict:=true` 时**拒绝** |
| `3` | 目标点在**本地地图盒外**（`±local_update_range`，`:340`） | 同 `2`（默认放行 + WARN；`strict` 时拒绝） |
| `4` | **视线被已知障碍挡住**（`:328`）：无人机→目标点直线上（跳过自身 `goal_check_radius` 内）采到占据格 | **硬拒绝**：`... [EGO] Goal (x, y) REJECTED: the straight line from the drone to it crosses KNOWN occupied space ...` |

**参数**（`advanced_param.xml` 与 `sim_gazebo.launch` 都已串成 `<arg>`，可命令行覆盖）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `fsm/goal_check_enable` | `true` | 总开关（`false` = 完全恢复上游行为，不检查） |
| `fsm/goal_check_radius` | `0.5` m | 目标点检查球半径（机体半径 + 膨胀 ≈ 0.1~0.3 m，0.5 有余量） |
| `fsm/goal_check_strict` | **`false`** | `true` = 连"未观测 / 盒外"也拒绝，见下方警告 |
| `fsm/goal_check_los` | `true` | 邻域无法验证时，直线被已知障碍挡住就拒绝（"目标点在墙后"这一类） |

> ⚠ **`strict` 默认必须是 `false`（别随手打开）**：本仿真的地图**只有占据信息、没有空闲信息** ——
> `grid_map.cpp::cloudCallback` 只把点云点 → 膨胀占据，**没有做射线投射**去把"射线穿过"的格子
> 标成空闲；原始占据层 `/drone_0_grid_map/occupancy` 实测**发布 0 个点**。于是没被点云打到的格子
> 全是 `UNKNOWN`（`min_occupancy_log_=logit(0.80)`，未观测 = `clamp_min_log_-unknown_flag_ = -2.0`
> ⇒ `getOccupancy()==0`）。实测**开敞空地上的 `(6.0, 1.2)` 也是 `515/515 UNKNOWN`** ⇒ `strict=true`
> 会把**盒内所有点**都拒掉（盒外的点也被拒）⇒ 无人机**基本不可指挥**。`strict=true` 只适合
> "地图有完整空闲信息"的场合（真机里用会做 raycast/projective 更新的栅格）。

> **为什么加 `los` 这一条**：`ego-planner` 只有**局部**规划器（B 样条优化），当目标方向被已知墙
> 挡死时容易掉进局部极小，直接返回一条**穿墙轨迹**（replan 日志里刷 `collided, keep optimizing`
> 随后 `plan_success=1`）。2026-10-10 复现：无人机在 `(7.2, 3.0)` 时把目标打到西墙后面的
> `(2.0, 4.0)`，规划器规划出穿墙轨迹，无人机撞在东墙/落地翻滚。所以"看不见目标 + 直线撞到
> **已观测到的**障碍"⇒ 直接拒绝。
> **绕过办法（推荐流程）**：先飞到**有视线**的位置（例如门洞前 `(4, 0)`），再点墙后的目标 ——
> 实测 `(4,0) → (7.0,2.5)` 的直线正好从门洞（`x=5, |y|<1.5`）穿过 ⇒ 放行并成功飞过去（下表 T5）。
> 要临时关掉这条：`roslaunch ego_planner sim_gazebo.launch flight_type:=1 goal_check_los:=false`。

⇒ 现在**点进墙里/点在墙后不会再撞墙**，只会看到一条警告 + 一个红球（或一条"放行但未验证"的提示）。

**原因 ②：RViz 里"看不见墙、也看不见飞机"。** `default.rviz` 原来只有 planner 的
`grid_map/occupancy_inflate`（稀疏体素，远看就是一片雾），既没有 FAST-LIO 点云显示项，
也因为没跑 `odom_visualization` 而让 `robot` / `path` 这两项**一直是空的** ——
等于在"盲点"。

**已修**：
- `default.rviz` 增加点云显示项 **`LiDAR(/cloud_registered)`**（橙色，`Use Fixed Frame: true`，
  与 planner 同系，1:1 显示墙的位置）；
- `sim_gazebo.launch` 增加 `odom_visualization` 节点（`frame_id=world`，
  `~odom` 重映射到 `/drone_0_visual_slam/odom`）⇒ RViz 里出现**机体 mesh + 飞行轨迹**；
- 默认视角 `rviz/ThirdPersonFollower`(斜视) → **`rviz/TopDownOrtho`(正俯视)**。

**为什么一定要正俯视**：`2D Nav Goal` 是把鼠标射线与 **z=0 水平面**求交得到 (x,y)，
和"你看上去点在哪儿"无关。斜视时点到"墙的上沿"，射线会插到墙**后面**去 ——
墙高 3 m，视线每偏垂直 1° 就偏 ~0.05 m，偏 20° 就是 ~8 m（足够从门洞偏到门框里）。
正俯视（Pitch=90°）时这个偏差为 0。所以：

1. 点目标前先确认视角是 `TopDownOrtho`（工具栏切 View）；
2. 拿不准就先用 **`Publish Point`** 工具点一下，`rostopic echo /clicked_point` 看 (x,y) 对不对；
3. 被拒绝的点会有 `[EGO] Goal (...) REJECTED: ...` 警告 + RViz 红球，看到就换个空地再点
   （"放行但未验证"的点则是 `ACCEPTED but NOT VERIFIED: ...`，能飞但没被点云确认过）。

**回归验证**（`./run_sim.sh --manual` 起好、飞机悬停后执行；下面 T1~T7 是 **2026-10-10 干净重启后逐项实测**结果）：

```bash
source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
pub() {   # pub <x> <y>
  rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
    "{header: {frame_id: 'world'}, pose: {position: {x: $1, y: $2, z: 1.0}, orientation: {w: 1.0}}}"
}
```

| # | 场景（当前位姿 → 目标） | 期望 | 实测 |
|---|---|---|---|
| T1 | 悬停 → `(5.0, 3.0)` 东墙本体 | 拒绝(`1`) | ✅ `[EGO] Goal (5.00, 3.00) REJECTED: 147/515 cells around it are occupied or outside the map`；`Triggered!` 不增加，飞机原地悬停 |
| T2 | `(1.0, 1.4)` → `(2.0, 4.0)` 空地**未知**点 | 放行(`2`) + 真飞过去 | ✅ `ACCEPTED but NOT VERIFIED: 515/515 cells ... UNKNOWN ... but the line of sight to it is clear` ⇒ 6 s 内飞到 `(2.05, 4.15)` 稳定悬停 |
| T3 | `(2.0, 4.1)` → `(7.0, 2.5)` 墙后（**上次坠机的原用例**） | 拒绝(`4`) | ✅ `REJECTED: the straight line from the drone to it crosses KNOWN occupied space ...`；飞机原地未动 |
| T4 | `(2.0, 4.1)` → `(4.0, 0.0)` 门洞前（有视线） | 放行(`2`) + 真飞过去 | ✅ 飞到 `(4.05, -0.04)` 稳定悬停 |
| T5 | `(4.0, 0.0)` → `(7.0, 2.5)` **穿门洞** | 放行(`2`) + 穿门 | ✅ 直线正好从门洞过 ⇒ 放行；`t=1→2` s 之间从 `(4.28, 0.18)` 到 `(6.05, 1.63)`（即 `x=5` 处 `y≈0.9~1.3`，在门洞 `|y|<1.5` 内）⇒ 安全到达 `(6.89, 2.48)` |
| T6 | `(7.0, 2.7)` → `(8.0, 6.0)` 柱子 `(8,4)` 后面 | 拒绝(`4`) | ✅ 被视线拒绝（直线正好蹭到 `(8,4)` 的柱子） |
| T7 | `(7.0, 3.3)` → `(9.0, 0.0)` 盒内空地，且 `goal_check_strict:=true` | 拒绝(`2`) | ✅ `[EGO] Goal (9.00, 0.00) REJECTED (fsm/goal_check_strict=true): 515/515 cells around it are UNKNOWN ...` |

> 注意 **T3 说明了旧版文档里"直接点墙后 `(7,2.5)` 也能飞"的说法已不成立**（那是靠规划器撞运气绕门洞，
> 换个位置就会规划穿墙轨迹）。现在的正确用法是 T4 → T5：先飞到门洞前拿到视线，再点墙后的点。
> 复现脚本：`/tmp/diag/regress.sh`（`bash regress.sh <x> <y> <观察秒数>`：发目标点 + 抽 `logs/ego_planner.log`
> 新增行 + 每秒打 Gazebo 真值），当天原始日志 `/tmp/diag/t1.log` ~ `t7.log`。

---

### 7.4 重启 planner 时**别**用 `pkill -f sim_gazebo.launch` —— 会顺手杀掉 FAST-LIO（2026-10-10 实测）

`run_sim.sh` 起的 4 个 roslaunch，命令行分别是：

```
roslaunch px4 mavros_posix_sitl.launch ...            # PX4 + Gazebo + mavros
roslaunch fast_lio mapping_sim_gazebo.launch          # <- 注意这里也含子串 "sim_gazebo.launch"
roslaunch ego_planner sim_gazebo.launch flight_type:=1
roslaunch to_mavros poscmd2mavros.launch ...
```

所以 `pkill -INT -f '[s]im_gazebo.launch'`（本意只是重启 planner）会**同时命中 `mapping_sim_gazebo.launch`**，
把 FAST-LIO 一起 `-INT` 掉 —— `logs/fast_lio.log` 里只会留一句 `[laserMapping-1] killing on exit`，很容易被忽略。

**后果（极易误判成"目标点闸门又失效 / 又撞墙了"）**：`/Odometry`、`/cloud_registered`、
`/drone_0_visual_slam/odom` 全部断流 ⇒ planner 日志一直 `[FSM]: state: INIT` + `no odom.`、
**本地地图是空的**；而 PX4 靠 Gazebo 真值还能维持 OFFBOARD 悬停一会儿 ⇒ 此时点**任何**目标点，
闸门都会报 `515/515 UNKNOWN`（因为地图里什么都没有）并"放行"，随后飞机在**无定位/无地图**状态下
越飘越远甚至翻滚坠地。**排查口诀**：看到 `no odom` 或闸门报"整球 UNKNOWN"，先跑 `rostopic hz /Odometry`
确认 LIO 还活着；`pgrep -af 'roslaunch fast_lio'` 应该非空。

**正确做法（只重启 planner）**：

```bash
pkill -INT -f 'roslaunch ego_planner sim_gazebo'          # 精确到包名+launch
sleep 4
pkill -9 -f '[e]go_planner_node'; pkill -9 -f '[t]raj_server'; pkill -9 -f '[p]cl_render_node'
cd /home/safina/keyan_project/something1017/TECH/sim_assets
source ./env.sh
nohup roslaunch ego_planner sim_gazebo.launch flight_type:=1 >> logs/ego_planner.log 2>&1 &
# 验证: rosparam get /drone_0_ego_planner_node/fsm/goal_check_los   # 应为 true
#       pgrep -af 'roslaunch fast_lio'                              # FAST-LIO 必须还在
```

要连 FAST-LIO 一起重启（比如怀疑定位漂了），就老老实实整体重来：`pkill -INT run_sim.sh` →
确认残留进程清干净（`roslaunch`/`gzserver`/`mavros`/`laserMapping`/`ego_planner_node` 全无）→
`./run_sim.sh --manual`（内部要等 PX4/Gazebo 30 s + FAST-LIO 20 s）。

---

## 8. 可选：让 PX4 只用视觉定位（真正验证 FAST-LIO）

> ⚠️ **先看这条**：在 SITL 里直接开 `vision2mavros`（`run_sim.sh --vision`）会让 PX4 报
> `Preflight Fail: Yaw estimate error` 而**拒绝解锁**。原因是 SITL 的 EKF2 已经有
> GPS + 磁力计给出的偏航，再喂一路 LIO 的偏航（FAST-LIO 的 `camera_init` 与世界系
> 只差一个常数偏航）就冲突了。要开就得同时按下面把 EKF2 的融合源配好。
> **只想跑通"感知→规划→控制"链路的话，默认（不开 vision）就够了**——
> `ego-planner` 是直接吃 FAST-LIO 的 `/Odometry`，不经过 mavros。

默认 PX4 SITL 用 GPS+气压，所以 `vision_pose` 只是"顺便喂进去"。
想让 PX4 **真的靠 LIO 定位**（更接近真机），在 PX4 的 MAVLink shell / QGC 里：

```
param set EKF2_GPS_CTRL 0      # 关掉 GPS
param set EKF2_EV_CTRL 7       # 用外部视觉: 水平位置+垂直位置+速度 (需含 EV_CTRL_YAW 才吃偏航)
param set EKF2_HGT_REF 3       # 高度参考用 vision
param set EKF2_EVP_NOISE 0.05  # 按需
param set EKF2_EVA_NOISE 0.1
```

或者从 mavros 用 `param set`（注意 float 参数要用 `value.real`，直接
`rosrun mavros mavparam set` 可能因为类型不匹配报 `OSError: Request failed`）：

```bash
rosservice call /mavros/param/set "{param_id: 'EKF2_GPS_CTRL', value: {integer: 0, real: 0.0}}"
rosservice call /mavros/param/set "{param_id: 'EKF2_EV_CTRL',  value: {integer: 7, real: 0.0}}"
rosservice call /mavros/param/set "{param_id: 'EKF2_HGT_REF',  value: {integer: 3, real: 0.0}}"
rostopic hz /mavros/vision_pose/pose     # 必须先确认它稳定在 ~30 Hz
```

> 注意：关掉 GPS 后如果 `vision_pose` 没在稳定发布，PX4 的 EKF 会失锁，无法解锁/起飞。

---

## 9. 已知限制 / 还没做的事

- 目前只做了 **单机**。`lidar3d.sdf` 里话题名、以及 `ego_planner/sim_gazebo.launch`
  里的 relay 都是写死 `drone_0` 的；多机需要模板化（每机一套 LiDAR/IMU + 一套 FAST-LIO 实例）。
- `poscmd2mavros` 用的是 **位置+速度前馈+yaw** 的外环（`control_mode:=0`），
  直接吃 PX4 的位置环；没有实现 px4ctrl 那套加速度前馈 + `kx/kv` 自整定控制器。
  如果后面要更贴近真机控制效果，可以在此基础上扩展。
- `control_mode:=1`（速度+yaw_rate）的坐标转换已按实测规则处理（yaw_rate 会被
  mavros 取反），但没有做长时间飞行验证，默认仍推荐 `control_mode:=0`。
- **`poscmd2mavros` 的悬停保持会缓慢漂移**（2026-10-10 观察，与目标点闸门无关）：
  到达目标点后的 `WAIT_TARGET` 期间飞机每秒漂 ~1~2 cm（实测 `(2.05, 4.15, 0.98)` 过 14 s 变
  `(2.07, 4.17, 1.12)`），贴墙悬停时这份漂移会让飞机慢慢蹭到墙；另外实测它会在长时间没有
  `pos_cmd` / 丢 OFFBOARD 后**自动重新解锁 + 起飞**（一路爬升到墙顶高度）。
  待单独处理：给 `poscmd2mavros` 加"目标点到位后锁死 setpoint / 不再自动解锁"的逻辑。
- 静态 TF `world -> camera_init` 是**按"起点即原点"的假设**给的单位变换；
  如果以后做"中途接入 LIO"，要改成真实变换。
- 传感器的外参（`extrinsic_T/R`）在 `sim_gazebo.yaml` 里是单位阵
  （模型里 IMU 与 LiDAR 原点已重合，所以这是精确的）；
  如果以后把 IMU/LiDAR 挪位置，记得同步改模型里的 `<pose>` 和 yaml。
- **整条链路已在本机实测跑通**（2026-10-10）：PX4 SITL(v1.14.3) + Gazebo Classic 11.15.1
  + FAST-LIO + ego-planner-swarm + poscmd2mavros。两种飞行方式都实测过：
  ① 预设航点 `(8,0,1) → (-8,0,1)` 自动飞并悬停（第 6.5 节；**注意它有"第一段被空跑"
  的时序竞态，见 7.2**）；
  ② **RViz `2D Nav Goal` 手动连点两个目标点** `(6,0,1) → (-6,0,1)`，到点后稳定悬停、
  还能继续接收下一个目标（第 6.6 节）；RViz 的**目视核对**已在 2026-10-10 补做：
  Fixed Frame `world`、`world->camera_init` 单位 TF、`/cloud_registered` 点云里墙在 x=±5、
  机体 mesh 与轨迹显示项都已接通并核对过（排查过程见 **7.3**）。
  尚未做的是：多机、以及 EKF2 只用视觉（第 8 节）那条路径的完整飞行验证。
