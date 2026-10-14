# Swarm-Formation → TECH/src 合并说明 (Option B)

日期: 2026-10-12
方案: **Option B** —— 把 Swarm-Formation(SF) 的包平铺到 `TECH/src`,并用 SF 版本**替换** `ego-planner-swarm`(EP) 的同名包,合并成单一规划栈。

---

## 1. 目标结构

* 编译工作区(**未改动**): `TECH/src/ego-planner-swarm`（`sim_assets/env.sh` 里的 `TECH_WS`）
* SF 各包的**物理位置(平铺后)**: `TECH/src/planner/*`、`TECH/src/uav_simulator/*`、`TECH/src/Utils/*`
* 工作区 `src/` 里用**绝对软链接**(沿用原有 `FAST_LIO`/`TO_MAVROS` 风格)指向上述平铺包

```
TECH/src/
├── planner/{path_searching,plan_env,plan_manage,traj_utils,traj_opt,swarm_bridge,swarm_graph}   ← SF 包(物理)
├── uav_simulator/{fake_drone,local_sensing,map_generator,mockamap,so3_control,so3_quadrotor_simulator}
├── Utils/{odom_visualization,pose_utils,quadrotor_msgs,rviz_plugins,uav_utils}
├── Swarm-Formation/            ← 原 clone 的壳(包已被移出;仅剩 .git/README/Dockerfile/Makefile)
└── ego-planner-swarm/          ← 编译工作区
    └── src/
        ├── planner/  bspline_opt(d) drone_detect(d) rosmsg_tcp_bridge(d) + 7 个软链接
        └── uav_simulator/  6 个软链接 + Utils/{cmake_utils,multi_map_server,waypoint_generator}(d) + 5 个软链接
```

## 2. 被 SF 替换的 EP 同名包 (15)

`path_searching, plan_env, plan_manage(ego_planner), traj_utils, poscmd_2_odom, local_sensing_node,
map_generator, mockamap, so3_control, so3_quadrotor_simulator, odom_visualization, pose_utils,
quadrotor_msgs, rviz_plugins, uav_utils`

## 3. 保留(EP 独有)与排除

| 包 | 处理 | 原因 |
|---|---|---|
| `cmake_utils`, `multi_map_server`(含编译修复), `waypoint_generator` | **保留** | SF 没有;`multi_map_server/CMakeLists.txt` 的 `multi_map_server_generate_messages_cpp` 修复也保留 |
| `drone_detect` | **保留** | 自洽,不依赖被替换的包 |
| `bspline_opt` | **CATKIN_IGNORE** | 它 include `plan_env/obj_predictor.h`,而 SF 的 `plan_env` 无此头;且 EP 的 ego_planner 已被替换 → 已成孤儿 |
| `rosmsg_tcp_bridge` | **CATKIN_IGNORE** | 它 include `traj_utils/{MultiBsplines,Bspline}.h`,SF 的 `traj_utils` 是 `PolyTraj/Assignment` → 不兼容;SF 用 `swarm_bridge` 取代 |

新增:`traj_opt`, `swarm_bridge`, `swarm_graph`(链接进 `src/planner/`)。

## 4. 保留的自定义启动/配置(来自 EP plan_manage)

已复制进合并后的 `TECH/src/planner/plan_manage/launch/`:

* `sim_gazebo.launch`   ← 你的 PX4/Gazebo 单机仿真启动文件。其中 traj_server 的 remap 已从
  `~planning/bspline` 改为 SF 的 `~planning/trajectory`(+ `start`/`finish`)。
* `default.rviz`        ← 你的定制版(3080 行)
* `advanced_param.ego-planner-swarm.xml` ← EP 版 advanced_param.xml,作参考保留
* `advanced_param.xml`  ← 仍用 SF 版,但**追加声明**了 `point4_x/y/z`、`goal_check_strict`、
  `goal_check_los` 三个 arg(带默认值),这样 `sim_gazebo.launch` 能正常解析。

## 5. 编译

```bash
source /opt/ros/noetic/setup.bash
cd /home/safina/keyan_project/something1017/TECH/src/ego-planner-swarm
catkin_make -j4          # 本次结果: CATKIN_EXIT=0, 25 个包, 147 个 target
```

产物:`devel/lib/ego_planner/{ego_planner_node,traj_server}` 等。

## 6. 运行

* 单机(Gazebo/PX4,沿用你的脚本):`sim_assets/run_sim.sh` → `roslaunch ego_planner sim_gazebo.launch` + `rviz.launch`
* SF 自带编队仿真:`roslaunch ego_planner normal_hexagon.launch`(已验证可解析:`drone_999_bridge_node`、`random_forest`、5 机编队等)

## 7. 回滚(随时可退)

```bash
cd /home/safina/keyan_project/something1017/TECH
# (a) 只恢复被替换/移动的 EP 包
cp -a _backup/removed_ego_packages/planner/.        src/ego-planner-swarm/src/planner/
cp -a _backup/removed_ego_packages/uav_simulator/.  src/ego-planner-swarm/src/uav_simulator/
cp -a _backup/removed_ego_packages/Utils/.          src/ego-planner-swarm/src/uav_simulator/Utils/
# 注意: 覆盖前先删掉对应软链接
# (b) 或整仓还原(含你的自定义改动)
rm -rf src/ego-planner-swarm && tar -xzf _backup/ego-planner-swarm_20261012_0930.tar.gz -C src/
```

备份内容:

* `_backup/ego-planner-swarm_20261012_0930.tar.gz` —— 合并前的完整源码(含未提交自定义,不含 build/devel)
* `_backup/ego-planner-swarm_custom_changes_20261012_0930.patch` —— 你的未提交改动 diff
* `_backup/custom_fsm/` —— 定制过的 `ego_replan_fsm.{h,cpp}`(goal-safety 代码, +206 行)
* `_backup/ego_plan_manage_launch/` —— 原 EP plan_manage/launch 全部文件
* `_backup/removed_ego_packages/` —— 被替换下来的 EP 同名包源码

## 8. 已知后续项(需要时再处理)

1. **goal-safety 代码未移植**。EP 的 `ego_replan_fsm.cpp::checkGoalSafety()`(+206 行,含 `goal_check_*`)
   不在 SF 的 fsm 里(两者是不同规划器)。已备份在 `_backup/custom_fsm/`;如需该功能要手工移植。
2. **运行时语义**:SF 的 ego_planner 是编队/poly_traj 版,与 EP 的 EGO-Swarm/bspline 版参数不完全对应。
   `sim_gazebo.launch` 仍用 `flight_type=2` 预设航点;若只用单机,建议按需在 `advanced_param.xml` 调参。
3. **SF 是 melodic 时代仓库**(其 `src/CMakeLists.txt` 指向 `/opt/ros/melodic/...`),
   在 noetic 下虽编译通过,若遇到运行期 API 差异需自行适配。
4. `devel/lib/libbspline_opt.so`、`devel/lib/rosmsg_tcp_bridge/bridge_node` 是忽略前遗留的**陈旧产物**,
   不影响编译;要彻底清理可 `catkin_make clean && catkin_make`。
5. `sim_assets/env.sh` / `run_sim.sh` **无需修改**(工作区路径未变)。
6. SF 的 `normal_hexagon.launch` / `run_in_sim.launch` 依赖 SF 自带的 `map_generator`/`mockamap`/
   `pcl_render_node` 仿真器;与 PX4/Gazebo + FAST-LIO 的真实仿真链路是两套,按需选择。

---

## 7. 后续: 多机编队 (已实现并验证)

在合并结果之上又接了一条**多机编队**链路: 一个 Gazebo 里 N 架 PX4 SITL, 每架各跑一份
FAST-LIO 做状态估计, 全部用合并后的 Swarm-Formation 做编队规划 + 机间避让, RViz 里点一次
"2D Nav Goal" 整队按阵型飞过去 (实测 3 架, 阵位误差 < 8 cm)。

* 一键启动: `sim_assets/run_formation.sh 3` (新 catkin 包 `sim_utils`) 
* 详细说明 (架构图 / 数据流 / 8 个关键坑 / 调参 / 限制): **`MULTIDRONE_FORMATION_README.md`**
* 对既有代码的改动只有向后兼容的两处: `TO_MAVROS/src/poscmd2mavros.cpp` 增加
  `origin_x/y/z` (默认 0, 单机行为不变), `sim_assets/env.sh` 增加 `export SIM_ASSETS_DIR`。
* 单机链路 (`run_sim.sh` + `sim_gazebo.launch`) 未受影响, 已回归验证仍能正常解析。
