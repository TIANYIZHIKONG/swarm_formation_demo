# 多机编队飞行 OOM 修复 — 验证报告

日期: 2026-10-12
工作区: `/home/safina/keyan_project/something1017/TECH`
启动方式: `sim_assets/run_formation.sh 3 --headless --no-gen`

---

## 1. 故障现象与根因

**现象**：编队飞行时 3 个 `ego_planner_node` 中有 2 个被 OOM-killer 杀掉，
每次都是稳定的单次巨量分配，`dmesg` 记录（修复前，12:05:53）：

```
Out of memory: Killed process 114618 (ego_planner_nod)
total-vm:39089576kB, anon-rss:23741172kB   # ≈ 38.1 GB 单进程
```

**根因链**（100% 定位）：

1. `GlobalTrajData::duration` / `LocalTrajData::duration` 等字段**未初始化**，
   存在“从未被赋值就被读取”的路径（A* 失败 / 优化器 force return / 收到广播轨迹前）。
2. FSM 用该值直接给容器定容：
   `int i_end = floor(duration / 0.1); std::vector<Eigen::Vector3d> v(i_end);`
   合计 3 处调用点（`ego_replan_fsm.cpp` 原 `:410-411`、`:666-667`、`:722-723`）。
3. 垃圾 `duration`（实测约 1.607e9）→ 约 1.6e9 个 `Eigen::Vector3d`（24 B）
   → **单次 ~38 GB 分配** → OOM。

修复策略：**零初始化 + 入口校验 + 产出校验 + 采样硬上限**（四道防线）。

---

## 2. 改动清单

| 文件 | 位置 | 内容 |
| --- | --- | --- |
| `src/planner/traj_utils/include/traj_utils/plan_container.hpp` | `:18,23,28,...` | `GlobalTrajData` / `LocalTrajData` / `MultiTrajData` 全字段零初始化（`double duration{0.0}`、`int drone_id{-1}`、`Eigen::Vector3d start_pos{...Zero()}` …） |
| `src/planner/plan_manage/include/plan_manage/ego_replan_fsm.h` | `:74-76` | 新增上限常量 `max_goal_distance_{1.0e4}`[m]、`max_global_duration_{1.0e4}`[s]、`max_global_path_vis_pts_{100000}`（1e5×24 B = 2.4 MB） |
| 同上 | `:122` | 声明 `bool sampleGlobalTrajForVis(std::vector<Eigen::Vector3d>&)` |
| `src/planner/plan_manage/src/ego_replan_fsm.cpp` | `:651-680` | 新增 `sampleGlobalTrajForVis()`：`isfinite(duration) && 0 ≤ duration ≤ 1e4` 校验，超限报错并**跳过可视化**（返回 false）；采样点数硬上限 1e5；是 3 处 `resize` 的唯一替代 |
| 同上 | `:420 / :749 / :798` | 3 个原不安全调用点全部改为 `if (sampleGlobalTrajForVis(gloabl_traj))` |
| 同上 | `:686-730` | `formationWaypointCallback` 三重护栏：① odom 就绪且有限 ② goal 有限（NaN/Inf 拒绝）③ `end_pt_` 有限且 `‖end_pt_-odom_pos_‖ ≤ 1e4 m` |
| 同上 | `:487` | 广播轨迹回调 drone_id 越界检查（0–49），非法丢弃 |
| 同上 | `init()` / 变量声明 | `have_odom_`、`odom_pos_` 等状态初始化；`swarm_relative_pts_[50][3]` 初始化循环补全为 50 项（默认 0.0）；补 `#include <cmath>` |
| `src/planner/plan_manage/include/plan_manage/planner_manager.h` | `:66-67` | `max_global_traj_dist_{1.0e4}`、`max_global_traj_duration_{1.0e4}` |
| `src/planner/plan_manage/src/planner_manager.cpp` | `:416-426` | `planGlobalTrajWaypoints()` **产出后**校验 `getTotalDuration()`：非有限/负/超 1e4 s → 报错并 `return false`（不调用 `setGlobalTraj`），使 `duration` 保持 0.0 |
| 同上 | `:361` | `max_vel_` 合法性输入校验；补 `#include <cmath>` |

编译：全工作区 `catkin_make` **0 error**，`ego_planner_node` 重新链接。
（注：`traj_utils/traj_opt/plan_manage` 是 `src/planner/*` 的符号链接，`--pkg` 单包编译不可用，须全量 `catkin_make`。）

---

## 3. 验证结果

### 3.1 内存（核心指标）

| 指标 | 修复前 | 修复后 |
| --- | --- | --- |
| 单进程峰值 VSZ | 38.15 GB（dmesg `total-vm:39089576kB`） | **1.28 GB（1309 MB）恒定** |
| RSS | ~23 GB（anon-rss） | **823 MB / 823 MB / 822 MB** |
| 存活 planner 数 | 1/3（2 个被 OOM 杀） | **3/3，持续 >10 min** |
| 修复后 `dmesg` OOM 记录 | — | **0 条新增**（仅剩 12:05:53 那条修复前记录） |

`/tmp/verify_fix.log` 全时段采样 `maxVSZ=1309MB`，无任何增长尖峰。

### 3.2 编队收敛（功能验证）

发布目标 `(25, 0, 0.5)` 到 `/move_base_simple/goal` 后，
FSM 迁移 `WAIT_TARGET → SEQUENTIAL_START → EXEC_TRAJ`（3 架各一次），
`REPLAN_TRAJ ↔ EXEC_TRAJ` 重规划循环正常，最终实测位置：

| 无人机 | 实测 (x, y, z) | 目标槽位 | 误差 |
| --- | --- | --- | --- |
| drone 0 | (25.016, -0.068, 0.528) | (25, 0) | 6.8 cm |
| drone 1 | (28.337, -2.058, 0.436) | (28.46, -2) | 12.3 cm |
| drone 2 | (25.040, -4.012, 0.497) | (25, -4) | 4.1 cm |

→ **三机全部到达编队槽位，编队飞行成功**。

### 3.3 护栏静默性

正常编队飞行期间，`formation.log` 中
`Goal rejected` / `Abnormal global trajectory duration` / `invalid drone_id` /
`abnormal global trajectory duration` / `invalid max_vel_` **均未出现**（零误报）。

---

## 4. 护栏压力测试（主动触发）

脚本 `/tmp/stress_guard.py`、`/tmp/far_legal_goal.py` 向 `/move_base_simple/goal`
注入异常目标，观测内存与日志：

| 注入目标 | 结果 | 日志证据 |
| --- | --- | --- |
| (1e6, 1e6, 0.5) | 3/3 拒绝 | `[drone N] Goal rejected: bad formation target [1000000.0, ...]` |
| (nan, 0, 0.5) | 3/3 拒绝 | `Goal rejected: non-finite goal position (nan, 0.0)` |
| (inf, 0, 0.5) | 3/3 拒绝 | `Goal rejected: non-finite goal position (inf, 0.0)` |
| (2e4, 0, 0.5) | 3/3 拒绝 | `Goal rejected: bad formation target [20000.0, ...]` |
| **(9999, 0, 0.5)（合法但极远）** | **走正常通路，无告警** | 生成约 5e4 个采样点（≈1.2 MB），**VSZ 全程恒为 1309 MB 零增长** |

最后一行是关键对照：同一量级的采样在旧代码中由无上限 `resize` 完成，
在垃圾 `duration` 下就是 38 GB；现在即使合法大目标也只占 1.2 MB，
而异常 `duration` 会在 `sampleGlobalTrajForVis` / `planGlobalTrajWaypoints`
两处被拦截并回落为 0.0。

---

## 5. 过程中发现的**既有**问题（与本次 OOM 无关，未改动）

启动后立刻发布目标会失败并悬停，日志：

```
[WARN]  drone 0 is too close to the obstacle at relative time 0.325135!
[ERROR] Unable to handle the initial or end point, force return!
[WARN]  Emergency stop! time=0.970000
[ERROR] Ran out of pool, index=405 -1 36      (A* 资源池耗尽)
[FSM]:  from EXEC_TRAJ to WAIT_TARGET          (放弃目标 → 原地悬停)
```

原因：起飞点 (~0.5 m 高) 在局部地图中被判为“贴近障碍/不可通行”，
优化器对起点/终点无法处理 → 紧急停止 → FSM 清空目标并悬停，
`traj_server` 因收不到 B 样条而不发布 `pos_cmd`。

**规避方式**：等 FAST-LIO 建图稳定（本机实测约起飞后 60–90 s）再发目标；
本次验证即为“等地图稳定后重发 (25,0)”后立刻成功。
如需彻底解决可考虑调高起飞高度 / 增大 `obstacle_clearance` /
在 FSM 中把“起点不可通行”改为“先垂直爬升再规划”。

---

## 6. 回归验证方法

```bash
# 1) 启动仿真（headless 便于监控）
cd /home/safina/keyan_project/something1017/TECH
sim_assets/run_formation.sh 3 --headless --no-gen

# 2) 等 ~60-90 s 建图稳定后发布编队目标 (25, 0)
rosrun formation_utils ...   # 或 rviz 里 2D Nav Goal；本文用 /move_base_simple/goal

# 3) 内存/存活监控（应恒为 ~1.28 GB 且 3/3 存活）
ps -eo pid,vsz,args | grep '[e]go_planner_node'

# 4) 护栏测试（应打印 Goal rejected，且内存不涨）
python3 /tmp/stress_guard.py

# 5) 确认无新增 OOM
dmesg -T | grep -iE 'oom|killed process'
```

监控脚本与日志：`/tmp/stress_guard.py`、`/tmp/far_legal_goal.py`、
`/tmp/verify_fix.log`、`/tmp/verify_fix_wrapper.log`、
`sim_assets/logs/formation.log`、`~/.ros/log/latest/rosout.log`。
---

# 追加（同日 14:1x–14:5x）：14:12 又发生一次同签名 OOM → 真正根因定位；附膨胀/安全间距审计

## 7. 重要更正：第 1–3 节的结论不完整

`dmesg` 中 **6/6** 次 OOM 的 `total-vm` 完全一致（±8 MB）：

| 时间 | pid | total-vm | anon-rss |
| --- | --- | --- | --- |
| 12:05:24 | 87035 | 39081516 kB | 23687684 kB |
| 12:05:53 | 114618 | 39089576 kB | 23741172 kB |
| … | … | 390815xx kB | … |
| **14:12:17** | **203513** | **39081504 kB** | **22961796 kB** |

14:12 这一次运行的**就是 12:23 那个"已修复"的二进制**，证据（`~/.ros/log/46b20de2-…/roslaunch-edianyun-202996.log`）：

```
[drone_2_ego_planner_node-29] process has died
  [pid 203513, exit code -9, cmd .../devel/lib/ego_planner/ego_planner_node ...]
```

`exit code -9` = SIGKILL = OOM-killer。⇒ `duration/0.1` 那 3 处虽然确实是隐患（已修），
**但不是这 39 GB 的来源**（它被 1e5 采样上限挡住后，OOM 依旧复现）。

## 8. 真正根因（定量闭环）

39.08 GB 的分配来自 `MinJerkOpt::reset()` 一族（`traj_opt/include/optimizer/poly_traj_utils.hpp:1128`）：

| 分配 | 字节 / piece |
| --- | --- |
| `T1.resize(N)`（VectorXd） | 8 |
| `A.create(6N, 6, 6)` → `6N×(6+6+1)` 个 double，且 **立即 `fill_n` 清零** | **624** |
| `b.resize(6N, 3)` | 144 |
| `gdC.resize(6N, 3)` | 144 |
| （case 2 前置）`innerPs(3, N−1)` | 24 |
| （case 2 前置）`piece_dur_vec`（VectorXd::Constant） | 8 |
| `Trajectory.reserve(N)` → N × `Piece`(8+144) | 152 |
| **合计** | **≈ 1104 B / piece** |

反推：实测分配 ≈ 39.08 GB − 基线 VSZ 1.267 GB = **37.8 GB** ⇒ `piece_num ≈ 3.4e7 ~ 4.1e7`。
而 `piece_num = ceil(‖start − local_target‖ / polyTraj_piece_length(2.0))`
⇒ **`‖start − local_target‖ ≈ 7e7 ~ 8e7 m`**（正常只可能是 0~7.5 m）⇒ 必然是垃圾值。
RSS 侧同样吻合：`A` 的 624 B/piece 被立即清零 ⇒ 4e7×624 ≈ 24 GB ≈ 实测 anon-rss 22.96 GB。

**垃圾值从哪来（probe3 日志给出的触发链）**

1. 目标点附近 A* 频繁失败：`[ERROR] Ran out of pool, index=359 70 -1`（每次 "reached goal" 前后若干条）；
2. `simple_path` 因此为**空**，而旧代码落入 `else // piece_num == 1` 分支去读 `simple_path[0] / simple_path[1]`
   → **空 vector 越界读（UB）**；用户 14:07 那次 `formation.log` 末尾正是两行 `Segmentation fault (core dumped)`；
3. 越界读到的垃圾 → `innerPts` / `time_vec` → `frontendMJ.generate()` → 垃圾初始轨迹 → `setLocalTraj()`；
4. 下一轮 `planFromLocalTraj()`：`start_pt_ = info->traj.getPos(t_cur)`（垃圾多项式外推）→ 巨值；
5. `computeInitReferenceState()` case 2：`piece_nums = ceil(‖start_pt_ − local_target_pt_‖/2.0)` ≈ 4e7
   → `MinJerkOpt::reset()` 一次 ~37 GB → OOM。

并行的第二个坑：`EGOPlannerManager::getLocalTarget()` 的两个输出
`local_target_pos/local_target_vel` 在 `global_start_time/duration` 非有限（NaN）时**两个分支都会被跳过**
→ 输出悬空；而 FSM 的 `local_target_pt_/local_target_vel_` 是未初始化 `Eigen::Vector3d`
→ 同样把垃圾直接喂给 `piece_nums`。本次一并堵住。

## 9. 本次新增的修复（叠加在 12:23 那四层之上）

| 文件 | 位置 | 内容 |
| --- | --- | --- |
| `traj_opt/src/poly_traj_optimizer.cpp` | `astarWithMinTraj()`，A* 调用之后 | **空路径早退**：`simple_path.size() < 2` → 报错 + `ctl_points.resize(3,0)` + `return`；**消除 `simple_path[0]/[1]` 越界读 UB（源头）** |
| `plan_manage/src/planner_manager.cpp` | `computeInitReferenceState()` case 1 | 校验 `traj.getPieceNum() > 0 && ctl_points.cols() > 0`，否则报错并 `return false`（空 MJO 不再被使用） |
| 同上 | `computeInitReferenceState()` case 2 | 校验 `local_target_pt/vel` 有限；`dist = ‖local_target − start‖ ≤ max_local_target_dist_(1e3 m)`；`piece_nums` 上限 `max_init_piece_nums_(200)`（正常只需要 ~4 段；200×2 m = 400 m 余量） |
| 同上 | `getLocalTarget()` | 入口即写 `local_target_pos = global_end_pt; local_target_vel.setZero();`（**输出永不悬空**）；`global_start_time/duration/end_pt` 非有限 → 报错返回；采样点非有限 → 返回；出口 `dist > 1e3 m` → 回落到 `global_end_pt` |
| 同上 | `reboundReplan()` 入口 | 校验 start/target 有限 + `target_dist ≤ 1e3 m`，否则 `return false`（不再把垃圾喂给 `piece_nums`） |
| `plan_manage/include/plan_manage/planner_manager.h` | `:69-80` | 新增 `max_local_target_dist_ = 1.0e3`、`max_init_piece_nums_ = 200` |
| `plan_manage/include/plan_manage/ego_replan_fsm.h` | `:92-98` | `end_pt_/end_vel_/local_target_pt_/local_target_vel_` **全部零初始化** |

编译：`catkin_make -j6` → **0 error**（`EXIT=0`），`ego_planner_node` 于 14:40 重新链接
（`strings ego_planner_node | grep -c 'implausible local target'` = 1，确认新守卫在二进制里）。

## 10. 验证

### 10.1 probe3 = 复刻用户触发时序（`/tmp/oom_probe3.sh`）

起飞后 **80 s** 就点 (25,0)（用户是 sim 89 s），随后 120/180/240 s 各再点一次：

| 指标 | 结果 |
| --- | --- |
| 到位 | 4 轮 × 3 架 = **12 次 `[drone N reached goal]`**，全部到位 |
| `Emergency stop` | **0** |
| `too close to the obstacle` | **1**（只在 sim 80.19 s，即"起飞期点目标"，与用户现象一致） |
| planner RSS 峰值 | **842 MB**（VSZ 1.27 GB），全程无尖峰 |
| 新增 dmesg OOM | **0**（最后一条仍是 14:12:17） |
| `Ran out of pool` | 18 次 → **现在被"空路径守卫"干净拒绝**（不再越界读），飞行照常完成 |

> 说明：本次没能复现出 39 GB（垃圾 `start_pt_` 的生成依赖具体时序），
> 但 39 GB 的两条可能通路（`piece_nums`、`duration/0.1`）现在都有硬上限 + 输入校验，
> 且源头 UB（空路径越界读）与"输出悬空"都已消除。

### 10.2 probe4 = 新配置下的干净编队飞行（`/tmp/oom_probe4.sh` + `/tmp/measure_clearance.py`）

见下方 (11.5) 的实测数字。



## 11. 机体/障碍物膨胀与安全间距审计（"还会有轻微碰撞"的原因）

### 11.1 机体真实尺寸（PX4 iris，`$PX4-Autopilot/.../models/iris/iris.sdf`）

| 部件 | 几何 | 备注 |
| --- | --- | --- |
| 机身 `base_link` | box **0.47 × 0.47 × 0.11 m** | 水平半宽 **0.235 m** |
| 旋翼 ×4 `rotor_i` | cylinder **radius 0.128 m**，安装于 **(±0.13, ∓0.22, 0.023)** | 桨叶半径 0.128（10 寸桨） |

⇒ 水平最大外伸 = 0.22 + 0.128 = **0.348 m**（y 向）；对角 √(0.13²+0.22²)+0.128 = **0.383 m**。
⇒ **有效的"整机半径"≈ 0.35 m**（最外缘是**桨尖**，不是机身），两机相撞需要中心距 **≥ 0.70 m**。

### 11.2 各项"膨胀/间距"的实际几何含义（原配置）

| 项 | 原值 | 几何含义 | 与机体对比 |
| --- | --- | --- | --- |
| 地图**硬**膨胀 `grid_map/obstacles_inflation` | 0.1 | `inf_step = ceil(0.1/0.1) = 1` → `occupancy_buffer_inflate_` 是 **±1 体素立方 = ±0.1 m**（`inflatePoint` 用"all inflate"分支）；加体素量化实际 ~**0.15 m** | **0.15 < 0.348 m ⇒ 硬检查根本挡不住桨尖**；A* 与 `getInflateOccupancy` 都用它 |
| 障碍软间隙 `optimization/obstacle_clearance` | 0.5 | ESDF (`updateESDF3d`) 从**膨胀后**的 `occupancy_buffer_inflate_` 算起 ⇒ 距真实面 ~0.65 m；代价 `w·(clearance−d)³`，**只在 CP `1 … 2/3` 生效** | 覆盖桨尖，但**是软代价**，小违反极便宜（d=0.35 时 cost=5e4·0.0034≈170） |
| 机间 `optimization/swarm_clearance` | 0.5 | 代码 `a=2.0`(z)、`b=1.0`(xy)：约束 `dz²/4 + dx²+dy² < (1.5·0.5)²` ⇒ 水平中心距 **0.75 m**、垂直 1.5 m；同样软代价、同样只在 CP ≤2/3 | **0.75 − 0.70 = 5 cm 余量** ⇒ 跟踪误差/相位滞后一来就"轻微碰撞" |
| FSM 机间紧急阈值 | `0.8 × swarm_clearance = 0.4 m`（`ego_replan_fsm.cpp:313`） | 只用于**事后**报警/急停 | **小于 0.70 m 的物理接触距离** ⇒ 报警时桨叶已经相交 |
| 轨迹检查覆盖率 | — | 优化器 `checkCollision()`（截到 `T_end`）与 FSM `checkCollisionCallback()`（截到 `t_2_3`）都只查轨迹**前 2/3** | 每段重规划轨迹的**后 1/3 无人检查**，终点/急停前后最容易擦碰 |

结论：**膨胀参数"数值上不小"，但"层级不对"**——
障碍物只有 0.1 m 的**硬**膨胀（< 机体半径），机间只有 0.75 m 的**软**约束（仅比物理接触距离大 5 cm），
所以"轻微碰撞"是设计余量为负/接近零的必然结果，而不是偶发 bug。

### 11.3 最终采用的配置（保守组合，已编译验证）

| 参数 | 文件 | 原值 → 新值 | 效果 / 理由 |
| --- | --- | --- | --- |
| `optimization/swarm_clearance` | `config/normal_hexagon.yaml` | 0.5 → **0.8** | 两机水平中心距 1.2 m ⇒ 桨尖余量 ~0.5 m（0.5 时只有 5 cm，就是"轻微碰撞"来源） |
| `optimization/obstacle_clearance` | 同上 | 0.5 → 0.5（**不变**） | 试过 0.6：本世界随机森林缝只有 0.8 m，0.6 会把"距真实面 ~0.95 m"变成**每处缝都强激活**的代价 → 实测把机体推离航线，撤回 |
| `optimization/weight_obstacle` / `weight_swarm` | 同上 | 5e4 → 5e4（**不变**） | 同上，避免"代价压过编队/可行性"导致轨迹发散 |
| `grid_map/obstacles_inflation` | `launch/advanced_param.xml` | 0.1 → **0.2** | 硬边界 ~0.25 m ≥ 机身半宽 0.235 m（**只加硬边界，不动软代价几何**，最安全） |
| FSM 机间紧急阈值 | `plan_manage/src/ego_replan_fsm.cpp:313` | `0.8*swarm_clearance`（0.4 m） → **`max(0.696+0.15, 0.8*swarm_clearance)` = 0.85 m** | 原来的 0.4 m **在物理接触距离（0.696 m）之内** —— 报警时桨叶已经相交 |

⚠️ 不要动的地方：`obstacles_inflation ≥ 0.4` 会把 0.8 m 的缝堵死；`obstacle_clearance ≥ 0.6` 或
`weight_obstacle ≥ 2e5` 会让障碍代价在本世界持续强烈激活（probe4 实测：drone2 被推离航线到
(12.5, 12.2) 并掉到 z=-0.335 m）。

### 11.4 若要"绝不接触"，还需要（未改，供选择）

1. 把障碍项从**软代价**升级为**硬约束**：对全部控制点做 `grid_map->getDistance(p) ≥ r_body+margin`
   检查，违反即 `return false`；并把检查覆盖到轨迹**后 1/3**（现在只查到 2/3）。
2. 把场景 `map_generator/min_distance` 从 0.8 m 提到 **≥1.6 m**：0.8 m 的缝对"含桨尖 0.7 m 宽"的
   机体本就不该穿（现在只是靠"软代价被违反"穿过去，桨尖难免蹭）。
3. 或降低编队密度（`swarm_scale`）/ 把起飞点与目标选在更空旷处。

### 11.5 实测（每轮都是 3 架 iris + 随机森林世界，`/tmp/measure_clearance.py`）

| 轮次 | 配置 | 机间最小距离 | 到位误差 | 告警 | 结论 |
| --- | --- | --- | --- | --- | --- |
| 上一节(旧配置, 12:33) | swarm_clearance 0.5 | 未测 | 4–12 cm | — | 三机到位，但理论余量仅 5 cm |
| **probe4** | 激进：0.6 / 0.2 / w=2e5 / swarm 1.0 | **1.395 m** | drone0 7.4 cm、drone1 6.9 cm、**drone2 20.4 m（掉地）** | too close 2、Emergency 0 | 机间余量大幅改善，但障碍项过强把 drone2 推离 |
| probe5 / probe6 | 激进 / 保守（无有效飞行） | — | — | 全 0 | **未起飞**：残留 roscore 触发 run_formation.sh 电量参数竞态（见 §13.1） |
| probe7 | 保守（无有效飞行） | — | — | Triggered! 0 | **目标未送达**：一次性发布器未 flush（见 §13.2） |
| **probe8（最终）** | 保守：0.5 / 0.2 / w=5e4 / swarm 0.8 | **3.701 m** | drone0 **0.181 m**、drone1 **0.117 m**、drone2 **0.490 m** | `Triggered!` 30、`reached goal` **3**、too close 5、**Emergency 0**、swarm distance 0、守卫误报 **0**、新 OOM **0**、RSS 842 MB×3 | **三机全部到达阵位，无急停、无 OOM、守卫零误报** |

> 说明：probe8 这一次三机航线基本平行（错时起飞），全程最近也有 3.70 m，所以"最小值"体现不出
> swarm_clearance 的效果；probe4 那次出现了 1.395 m 的接近（对应旧配置的理论边界 0.75 m）。
> 想量化你自己的场景，直接跑 `/tmp/measure_clearance.py <时长> 25 0 2.0` 即可。




### 11.4 若要"绝不接触"，还需要（未改，供选择）

1. 把障碍项从**软代价**升级为**硬约束**：对全部控制点做 `grid_map->getDistance(p) ≥ r_body+margin` 检查，
   违反即 `return false`；并把检查覆盖到轨迹**后 1/3**（现在截到 2/3）。
2. 把场景 `map_generator/min_distance` 从 0.8 m 提到 **≥1.6 m**：0.8 m 的缝对 0.7 m 宽的机体本就不该穿
   （现在只是靠"软代价被违反"穿过去的，桨尖难免蹭）。
3. 或降低编队密度（`swarm_scale` 减小）/ 把编队起点放到更空旷处。

### 11.5 probe4：新配置下的实测（见最终回复中的表）



---

## 12. 复现/回归脚本（本次新增）

| 脚本 | 用途 |
| --- | --- |
| `/tmp/oom_probe3.sh` | 复刻用户时序：起飞后 80/120/180/240 s 各点一次 (25,0)，监控 RSS + 抓 OOM |
| `/tmp/oom_probe4.sh` ~ `/tmp/oom_probe8.sh` | 干净编队飞行 + 量化测量（机间最小距离 / 到位误差 / 告警计数） |
| `/tmp/probe_goal2.py` | **可靠**目标发布器（latched + 连发 5 次） |
| `/tmp/measure_clearance.py` | 采样 3 机 odom，输出机间最小距离与相对阵位的到位误差 |
| `/tmp/bigalloc.c` | LD_PRELOAD 大分配抓捕器（>256 MB 的 malloc 打印调用栈；本次未用上，留作以后抓内存尖峰） |

## 13. 复现过程中发现的"仿真环境"坑（与本次修复无关，但会影响每一次实验）

1. **残留 roscore → 无人机永不起飞**（probe5/probe6 就是此因，两次都复现）：
   上一次运行留下的 `rosmaster` 仍在时，`run_formation.sh` 的 `set_battery_params()` 会看到
   **已死节点**残留的 `/drone_X/mavros/param/set` 服务注册，于是"立刻"执行参数设置 —— 但此时
   新 mavros 还没起，参数没真正进 PX4；解锁后 `SIM_BAT_DRAIN` 让电量跌破门槛 →
   `Preflight Fail: Battery unhealthy` → `COM_DISARM_PRFLT` 自动上锁 → 电机空转、**机体一直贴地**
   （实测：三机停在 z≈0.18 m、被自身推力推到离起飞点约 3.5 m 处，而 planner RSS 完全正常）。
   **规避**：每次重跑前 `pkill -f rosmaster; pkill -f rosout`（或 `rosnode cleanup`）。
2. **一次性脚本发目标会丢包**（probe5/6/7 全部命中）：`rospy.Publisher(..., queue_size=1)`
   发一条就退出，TCP 缓冲区未 flush → 目标压根没送达 FSM：日志里既没有 `Triggered!` 也没有任何拒绝
   （FSM 一直停在 `WAIT_TARGET`），很容易误判成"规划器坏了"。
   **规避**：`latch=True` + 连发多次（`/tmp/probe_goal2.py`），或在 RViz 里点 2D Nav Goal。
   （用 `rostopic pub` 复测：立刻出现 3 条 `Triggered!` → 说明 FSM/订阅完全正常。）
3. `~/.ros/log/.../rosout.log` 会被 Gazebo/RViz 的超长行（topic 列表）撑到几十 MB，
   排查请优先看 `sim_assets/logs/formation.log`（也可 `grep -v TF_REPEATED_DATA` 过滤刷屏）。

