# PX4 SITL 纯 SLAM 定位改造说明

> 目标：多机编队仿真里，把每架 PX4 SITL 的定位源从「模拟 GPS + 气压 + 磁力计」
> 改成「FAST-LIO 外部视觉（SLAM）」，即 PX4 侧 EKF2 完全靠
> `/mavros/vision_pose/pose` 定位，`EKF2_GPS_CTRL = 0`。

---

## 1. 定位链路（改造后）

```
Gazebo Classic: iris_lidar_<id>   3D LiDAR (/drone_<id>/points) + IMU (/drone_<id>/imu)
        │
        ▼
fast_lio  (fastlio_drone_<id>)          ──>  /drone_<id>/Odometry        (camera_init 系)
        │
        ├─────────────────────────────────>  /drone_<id>/cloud_registered
        │
        ▼
to_mavros / vision2mavros               ──>  /drone_<id>/mavros/vision_pose/pose
        │                                         │
        │                                         ▼
        │                                   mavros vision_pose 插件
        │                                         │
        │                                         ▼  VISION_POSITION_ESTIMATE (uORB)
        │                                   ekf2  ◀── 位置/高度/偏航全部来自这里
        │
        ▼
sim_utils / vio_offset.py               ──>  /drone_<id>_visual_slam/odom (公共 world 系)
        + /drone_<id>_pcl_render_node/cloud      │
                                                 ▼
                                ego_planner(Swarm-Formation) + traj_server
                                                 ▼
                                          /drone_<id>_planning/pos_cmd
                                                 ▼
                                to_mavros / poscmd2mavros  ──> mavros setpoint_raw/local
                                                 ▼
                                              PX4 EKF2 + OFFBOARD
```

关键点：**PX4 侧的定位（EKF2）和规划侧的定位（ego_planner）现在是同一个
FAST-LIO，完全一致，不再依赖任何 GPS。**

---

## 2. 改动清单

### 2.1 PX4 SITL 机型默认参数（核心）

机型文件（`PX4_SIM_MODEL=gazebo-classic_iris` → `SYS_AUTOSTART=10015`）：

| 文件 | 说明 |
| --- | --- |
| `/home/safina/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris` | 源文件（重新编译 PX4 会从这里拷到 build） |
| `/home/safina/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/10015_gazebo-classic_iris` | 实际运行的 rootfs（`~/.ros/sitl_iris_<i>/etc` 软链到这里） |

两份内容必须一致。新增的参数：

```sh
param set-default EKF2_GPS_CTRL 0    # bit0..3 全 0: 经纬/高度/速度/双天线航向都不融合
param set-default EKF2_HGT_REF 3     # 高度基准 = 视觉 (0=气压 1=GPS 2=测距 3=视觉)
param set-default EKF2_EV_CTRL 7     # 外部视觉: 水平位置+垂直位置+3D速度  (★ 不含偏航, 见 2.3)
param set-default EKF2_EV_DELAY 5    # 外部视觉延迟补偿 (ms)
param set-default ATT_EXT_HDG_M 1    # 外部(视觉)航向; EKF2 做估计器时不生效, 保留以对齐官方 vision 机型
param set-default COM_ARM_WO_GPS 1   # 允许无 GPS 解锁
param set-default LPE_FUSION 132     # 兼容 LPE (仅 SYS_MC_EST_GROUP 选 LPE 时生效)
```

与 PX4 官方自带的机型
`ROMFS/px4fmu_common/init.d-posix/airframes/1013_gazebo-classic_iris_vision`
（"3DR Iris Quadrotor SITL (Vision)"，官方用 `EKF2_EV_CTRL 15`）相比，
这里唯一的有意差别是 **`EKF2_EV_CTRL = 7`（不开外部视觉偏航）**，原因见 2.3。

### 2.3 偏航为什么不用视觉（`EKF2_EV_CTRL` 不带 bit3）

这是本次改造唯一"反直觉"的地方，必须写清楚，否则以后有人改回 15 会踩坑。

**现象**：`EKF2_EV_CTRL = 15`（含 bit3 外部视觉偏航）时，无人机**永远无法解锁**，
`formation.log` 里反复出现：

```
WARN [health_and_arming_checks] Preflight Fail: Yaw estimate error
```

**根因**（有 `.ulg` 证据）：

1. commander 的检查在
   `src/modules/commander/HealthAndArmingChecks/checks/estimatorCheck.cpp`：

   ```cpp
   if (!context.isArmed() && (estimator_status.mag_test_ratio > _param_com_arm_ekf_yaw.get()))
           // 默认 COM_ARM_EKF_YAW = 0.5
   ```

2. 而 `estimator_status.mag_test_ratio` 并不只是磁力计，EKF2 在
   `src/modules/ekf2/EKF/ekf_helper.cpp` 里把**外部视觉偏航也并了进去**：

   ```cpp
   #if defined(CONFIG_EKF2_EXTERNAL_VISION)
   if (_control_status.flags.ev_yaw) {
           mag = math::max(mag, sqrtf(_aid_src_ev_yaw.test_ratio));
   }
   #endif
   ```

3. `vision2mavros` 发出的偏航（由 FAST-LIO 四元数直接得到）对应的
   `estimator_aid_src_ev_yaw.test_ratio` 会**长时间保持 `inf`**
   （`innovation_variance = 0`）。于是 `mag_test_ratio = inf`，
   `inf > 0.5` 恒成立 → 检查永久失败 → 拒绝解锁。

   实测数据（一次 231 s 的飞行）：

   | 量 | 值 |
   | --- | --- |
   | `mag_test_ratio == inf` | 13496 / 28851 个采样，连续覆盖 **47.9 s ~ 155.8 s** |
   | `cs_ev_yaw` 置位时段 | **48.2 s ~ 155.2 s**（与上面完全重合）|
   | 把 `EKF2_EV_CTRL` 改成 7 之后 | `test_ratio` 立即恢复有限值（0.071），**15 s 内成功解锁并起飞** |

**所以偏航交给磁力计**（仍然与 GPS 无关）：

- 本 Gazebo 世界的磁北在 `+y`，机体朝 `+x` 时 EKF2 的 `heading ≈ +90°`；
  与规划侧 FAST-LIO（`camera_init` 系，偏航 ≈ 0）之间只是一个**固定 90° 偏置**。
- `poscmd2mavros` 内部按 `yaw_ned = pi/2 - yaw_enu` 做换算，且起飞时用
  `takeoff_yaw_enu_ = cur_yaw_enu_` 锁住当前偏航，位置控制完全不受影响。

**真想"连偏航也只信视觉"**（本 build 实测会卡在上面的 Yaw estimate error）：
```sh
param set-default EKF2_EV_CTRL 15
param set-default EKF2_MAG_TYPE 5    # 完全不用磁力计
```

各参数含义（源码：`src/modules/ekf2/ekf2_params.c`）：

| 参数 | 位/取值 | 含义 |
| --- | --- | --- |
| `EKF2_GPS_CTRL` | bit0 经纬 / bit1 高度 / bit2 3D速度 / bit3 双天线航向 | `0` = 完全不用 GPS |
| `EKF2_EV_CTRL` | bit0 水平位置 / bit1 垂直位置 / bit2 3D速度 / bit3 偏航 | `7` = 位置+高度+速度（**不含偏航**，见 2.3） |
| `EKF2_HGT_REF` | `0` 气压 / `1` GPS / `2` 测距 / `3` 视觉 | `3` = 高度以视觉为基准 |
| `EKF2_EV_DELAY` | ms（FLOAT，默认 0） | 外部视觉链路延迟补偿 |
| `ATT_EXT_HDG_M` | `0` 关 / `1` 视觉 / `2` 动捕 | 外部航向来源 |
| `COM_ARM_WO_GPS` | bool（默认就是 1） | 允许无 GPS 解锁 |

> 注意：`SENS_GPS_MASK` 是「多路 GPS 融合权重掩码」，**不是**关 GPS 的开关，
> 所以这里没有动它；真正关 GPS 融合的是 `EKF2_GPS_CTRL = 0`。

因为改动写进了机型文件的 `param set-default`，而各实例的
`~/.ros/sitl_iris_<i>/parameters.bson` 里**没有**保存过 `EKF2_*`
（只保存了 `CAL_*`/`SENS_*`/`SYS_AUTOSTART`/`MAV_SYS_ID`/`SIM_BAT_*` 等），
所以 `set-default` 一定会生效，不需要删参数文件。

### 2.2 启动脚本：每机一个 vision2mavros

| 文件 | 改动 |
| --- | --- |
| `src/sim_utils/launch/drone_all.launch` | 新增 `<arg name="vision_start_delay" default="42"/>` 和 "4.5) vision2mavros" 节点；每机一个，`FAST-LIO /drone_<id>/Odometry` → `/drone_<id>/mavros/vision_pose/pose` |
| `sim_assets/run_formation.sh` | 注释/就绪检查命令里补上 vision 相关话题 |
| `sim_assets/run_sim.sh` | 单机仿真：外部视觉**默认开启**（`START_VISION=1`），新增 `--gps` 关闭并把 `EKF2_GPS_CTRL/EKF2_HGT_REF/ATT_EXT_HDG_M` 改回 GPS 模式 |

`vision2mavros` 的配置要点：

- 输入 `/drone_<id>/Odometry`（FAST-LIO 原生 `camera_init` 系）。
- 节点内部会减掉前 100 帧的平均位姿 → 输出以「起始点（起飞点）」为原点，
  正好和 `poscmd2mavros` 用的 `origin_x/origin_y = (pos_x, pos_y)`、`origin_z = 0` 对齐。
- 输出 publisher 在 `to_mavros/src/vision2mavros.cpp` 里写死为**绝对**名字
  `/mavros/vision_pose/pose`，多机必须 `remap` 到 `/drone_<id>/mavros/vision_pose/pose`，
  否则 N 架会互相抢同一个话题。
- `body_lidar_{x,y,z}` = LiDAR/IMU 在机体上的安装偏移（与 `sim_assets/scripts/gen_drone_sdf.py`
  里的 `<pose>` 一致，即 `mount_x=0.05, mount_z=0.20`）。节点用它做
  `p_body = p_lidar - R * t_body_lidar`，把雷达位姿换算成机体位姿。

---

## 3. 怎么验证

```bash
source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
./run_formation.sh 1 --headless        # 先拿 1 架验证, 再上 3 架

# 1) PX4 侧参数是否真的变了
rosservice call /drone_0/mavros/param/get "param_id: 'EKF2_GPS_CTRL'"   # value.integer: 0
rosservice call /drone_0/mavros/param/get "param_id: 'EKF2_HGT_REF'"    # value.integer: 3
rosservice call /drone_0/mavros/param/get "param_id: 'EKF2_EV_CTRL'"    # value.integer: 7

# 2) 外部视觉是否在喂 PX4
rostopic hz   /drone_0/mavros/vision_pose/pose        # ~30 Hz
rostopic echo /drone_0/mavros/local_position/pose     # EKF2 定位结果

# 3) 有没有定位相关 preflight 报错
grep -iE "preflight|yaw estimate|position estimate" sim_assets/logs/formation.log
```

用 ULog 进一步确认「GPS 不再被融合、外部视觉被大量融合」
（分析脚本已固化到仓库；必须用**系统 python**，别用 conda 的）：

```bash
source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
ULG=$(ls -t ~/.ros/sitl_iris_0/log/*/*.ulg | head -1)
/usr/bin/python3 sim_assets/scripts/ekf_flags.py         "$ULG"   # 融合标志位占比
/usr/bin/python3 sim_assets/scripts/ekf_ulog_timeline.py "$ULG"   # 标志位/比值时间线
```

期望结果（本次改造实测，1 架）：

| 量 | 改造前（模拟 GPS） | 改造后（纯 SLAM） |
| --- | --- | --- |
| `cs_gps` | ~99% | **0.0%** |
| `cs_ev_pos` | 0% | **81.8%**（其余 18.2% 是 FAST-LIO 还没出数据前的启动段）|
| `cs_ev_hgt` | 0% | **81.8%** |
| `cs_ev_yaw` | 0% | 0%（按设计交给磁力计）|
| `cs_mag_hdg` | ~100% | 98.6% |
| `mag_test_ratio` 最大值 | — | **0.074**（< `COM_ARM_EKF_YAW`=0.5，所以不会报 Yaw estimate error）|

闭环飞行实测（1 架，`./run_formation.sh 1 --headless`，然后发一个
`/move_base_simple/goal` → world 系 (3, 1, 1)）：

```
[poscmd2mavros] OFFBOARD+armed -> TAKEOFF to (0.00, -0.00, 0.50)
INFO  [commander] Takeoff detected
[poscmd2mavros] takeoff reached (z=0.30) -> TRACK
...
PX4(EKF2/视觉) 稳定在 x=3.00  y=1.00  z≈0.49，波动 < 3 cm
```

即：**整段飞行 PX4 的位置全部来自 FAST-LIO 外部视觉，GPS 一次都没参与。**

---

## 4. 排障 / 回退

| 现象 | 原因 & 处理 |
| --- | --- |
| `Preflight Fail: Yaw estimate error`（无法解锁）| `EKF2_EV_CTRL` 带了 bit3（外部视觉偏航）时，EKF2 把 `ev_yaw.test_ratio`（长期为 `inf`）并进 `mag_test_ratio`，使 `mag_test_ratio > COM_ARM_EKF_YAW(0.5)` 永久成立。改回 `param set-default EKF2_EV_CTRL 7` 即可（详见 2.3） |
| `Preflight Fail: no position estimate` | `vision2mavros` 没在跑 / 比 PX4 需要的时刻起得晚。确认 `/drone_<id>/mavros/vision_pose/pose` 有 ~30 Hz 输出，必要时调大 `vision_start_delay` |
| 想临时回到 GPS 定位 | 单机：`./run_sim.sh --gps`；编队：把机型文件里那段参数注释掉，或在启动后对所有 `/drone_<i>/mavros/param/set` 设回 `EKF2_GPS_CTRL=7 / EKF2_HGT_REF=1` |

回退 PX4 改动（PX4-Autopilot 是 git 仓库，改动前 `git status` 是干净的）：

```bash
git -C /home/safina/PX4-Autopilot checkout -- \
    ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris \
    build/px4_sitl_default/etc/init.d-posix/airframes/10015_gazebo-classic_iris
```

---

## 5. 已知隐患（与本次改造无关，但会互相影响）

- `vio_offset` 把 `mount_z`（0.20 m）同时加到点云和机体位置上，导致规划器看到的
  FAST-LIO z 比 Gazebo 真值高约 0.18–0.24 m（xy < 7 cm）。定位改到 SLAM 后，
  PX4 的位置也继承同一套 FAST-LIO 原点，所以对「机间相对几何」影响不大，
  但会影响绝对限高判断。
- 规划侧 `map_size_z = 5.0` 已留足 z 余量，短期不会触发 A* 的
  "Ran out of pool"。
- 机型文件里那段 `param set-default` 只在「该参数尚未被保存过」时生效。
  如果以后有人把 `EKF2_GPS_CTRL` 存进了 `~/.ros/sitl_iris_<i>/parameters.bson`，
  需要先删掉该实例的参数文件再启动。

