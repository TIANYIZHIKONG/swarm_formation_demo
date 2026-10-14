# 碰撞后 planner 节点"死掉" — 根因与修复验证报告

日期: 2026-10-12
工作区: `/home/safina/keyan_project/something1017/TECH`
复现场景: `sim_assets/run_formation.sh 3`（复杂障碍物 world，编队飞行）
相关日志: `~/.ros/log/f5d88416-c61e-11f1-a81a-35697657d6a7/rosout.log`（仿真 0 → 241.5 s）

---

## 1. 故障现象

复杂障碍物环境下，某架无人机靠近/擦碰障碍后，该机的 `drone_N_ego_planner_node`
**从此不再输出任何东西**（不再规划、不再发 `/planning/trajectory`、不再广播轨迹），
编队里其它飞机继续正常飞 —— 表现就是"这个 planner 节点死掉了，停止参与飞行规划"。

注意：**不是崩溃（没有 core / OOM），是进程卡死**（占住 CPU、单线程 `ros::spin()`
回调再也不返回）。

## 2. 证据（用户自己那次运行的 rosout.log）

按 `rosout.log` 里逐节点"最后一条消息时间"统计，只有两个 planner 提前静默：

| 时间 (s) | 节点 | 消息 |
| --- | --- | --- |
| 101.516 | `drone_1_ego_planner_node` | `ego_replan_fsm.cpp:333` "drone 1 is too close to the obstacle at relative time 0.279464!" |
| 101.516 | `drone_1_ego_planner_node` | `:370` Try to replan a safe trajectory / `:374` Plan success when detect collision. |
| **115.528** | `drone_1_ego_planner_node` | `dyn_a_star.cpp:241` **"Failed in A star path searching !!! 0.2 seconds time limit exceeded."** |
| **115.528** | `drone_1_ego_planner_node` | `dyn_a_star.cpp:273` **"I don't know why, but only try A* again"** |
| 115.5 → 241.5 | （该节点） | **再无任何输出**（其余节点照常记录到 241.5 s） |

`rosout.log` 全文中这两条各只出现 **1 次**，且**恰好是该节点的最后两条消息**：

```
$ grep -c "Failed in A star" <rosout.log>            # 1
$ grep -c "I don't know why" <rosout.log>            # 1
```

而 `dyn_a_star.cpp:273` 正是 `ROS_WARN("I don't know why, but only try A* again");`
这一行 —— 它的**下一行**就是 `while (!finish)` 那个死循环。A* 的返回值打印表明：
第二次（不带 ESDF 检查的）A* **成功了**（否则会再打一条 "Failed in A star"），
所以流程正好走进那个循环并永远出不来。

## 3. 根因链（100% 定位）

文件：`src/planner/path_searching/src/dyn_a_star.cpp`
函数：`AStar::astarSearchAndGetSimplePath()`

1. 无人机贴近障碍 → FSM 触发重规划，局部目标点落在障碍 0.2 m 带内/边缘；
2. 第 1 次 A*（`use_esdf_check = true`）在 0.2 s 时限内**失败**
   （目标点在 0.2 m 带内 → 该目标在 ESDF 检查下不可达）；
3. `AstarSearch` 返回 false，但**返回值被忽略**，`path = getPath()` 拿到的是
   *上一次成功搜索遗留的旧路径*（`gridPath_` 没清）；
4. 旧路径首点与本次 `start_pt` 相距 > 0.5 m → 走 `path[0]` 分支，用
   `AstarSearch(..., use_esdf_check=false)` 重试（原注释 "I don't know why…"）；
5. 重试**成功**，但这条原始路径是"只对膨胀占据层安全"的 —— 它的**终点**
   （以及尾巴）与障碍距离 < 0.2 m；
6. 随后的"简单路径"简化循环：

   ```cpp
   while (!finish) {
     for (int i = end_idx; i < size; i++) {
       ... 对 cut_start→path[i] 按 1 cm 采样，用 checkOccupancy_esdf() 判安全 ...
       if (is_safe && i == size-1) { finish = true; ... }
       else { end_idx = i; cut_start = path[end_idx-1]; ... }
     }
   }
   ```

   当**最后一个点**和 `path[size-2]` 连不通时，循环会带着**完全相同的状态**
   （`end_idx == size-1`、`cut_start == path[size-2]`）重进 for 循环，判据一模一样、
   结果一模一样 → **死循环**。因为 planner 是单线程 `ros::spin()`，整个节点就此卡死。

### 同一段代码里另外两个隐患（碰撞后同样会被触发）

* `path[0]` 在 A* 失败（`getPath()` 为空）时被无条件读取 —— 空 vector 越界（UB），
  正是"为什么会走进重试分支"的不确定来源；
* `int check_num = ceil(norm/0.01)` 可能为 0 → `1.0/0` = Inf → 采样点变 NaN →
  `getDistance(NaN)` 的整数索引转换是 UB；`cut_start` 异常时 `check_num` 还可能是
  天文数字 → 又一个"卡死"来源。

## 4. 改动清单

| 文件 | 位置 | 内容 |
| --- | --- | --- |
| `src/planner/path_searching/src/dyn_a_star.cpp` | `AstarSearch()` 入口 `:121-131` | 新增 `gridPath_.clear();` —— A* 失败时 `getPath()` 返回**空**，不再回吐上次的旧路径 |
| 同上 | `astarSearchAndGetSimplePath()` `:270-308` | ① 入口校验 `start_pt/end_pt` 有限、`step_size > 0`，否则直接返回空（碰撞后 odom 可能是 NaN/Inf）；② 用 **A* 的返回值**驱动重试，替换 `(path[0]-start_pt).norm() > 0.5` 这种会越界读的判据；③ 路径为空时打 `ROS_ERROR` 直接返回空（上游 `astarWithMinTraj()` 本来就会拒收空控制点） |
| 同上 | 简化循环 `:322-392` | ① 外层 `while` 改为 `while (!finish && guard++ < size + 2)` 硬上限；② for 扫完仍 `!finish` 时**接受终点并结束**（附 `ROS_WARN`），保证一定终止；③ `check_num` 钳位到 `[1, 10000]`（防 `1/0`→NaN、防天文数字采样） |

设计原则：**宁可给出一条"最后一小段没清干净"的路径（交给后端优化器/下一轮重规划），
也绝不能让单线程节点卡死**。健康路径上的行为完全不变（见第 5 节场景 2）。

## 5. 验证

### 5.1 编译

```bash
source /opt/ros/noetic/setup.bash
cd /home/safina/keyan_project/something1017/TECH/src/ego-planner-swarm
catkin_make -j6            # 100% Built target ego_planner_node / traj_server，无 warning/error
ls -l devel/lib/ego_planner/ego_planner_node devel/lib/libpath_searching.so
```

（`path_searching / plan_env / plan_manage / traj_opt` 都是 `src/ego-planner-swarm/src/planner/`
下的软链接，指向 `TECH/src/planner/*`，所以只改一处即可。）

### 5.2 死循环复现器（确定性单元验证）

`/tmp/astar_simple_path_repro.cpp` 把旧/新两版循环的**状态机原样抄写**，只把
`checkOccupancy_esdf()` 换成桩函数，于是两者的唯一差别就是这次修复本身：

```bash
g++ -O1 -std=c++14 -Wall -o /tmp/astar_repro /tmp/astar_simple_path_repro.cpp && /tmp/astar_repro
```

实测输出：

```
scenario 1: goal inside the 0.2 m obstacle band (path_size=12)
  [ PASS] OLD code does NOT terminate  <-- the freeze that killed the node
        old loop rounds = 100001          <-- 撞到测试用的安全计数器才会停
  [ PASS] NEW code terminates
        new loop rounds = 1               <-- 1 轮就出来
  [ PASS] NEW code terminates within its round budget
  [ PASS] NEW code keeps the start point
  [ PASS] NEW code keeps the goal point
  [ PASS] NEW code never grows the path
scenario 2: blocked band in the middle, goal clear (path_size=40)
  [ PASS] both variants terminate
  [ PASS] NEW result identical to OLD on a passable blocked path (no regression)
  [ PASS] identical round count (no regression)
  [ PASS] NEW still ends at the goal
scenario 3: zero length segment between two waypoints
  [ PASS] NEW code terminates on a zero length segment
  [ PASS] NEW code still ends at the goal
scenario 3b: zero length segment + blocked tail (division by zero route)
  [ PASS] NEW code terminates
  [ PASS] NEW code terminates within its round budget
  [ PASS] NEW code still ends at the goal
scenario 4: NaN waypoint inside the path
  [ PASS] NEW code terminates with NaN inside the path

ALL CHECKS PASSED (0 failure(s))
```

* 场景 1 = 本次空难的确切形态：旧代码 **100001 轮仍未结束（即死循环）**，新代码 **1 轮**收工；
* 场景 2 = 回归保护：中间有一段"绕得过去"的障碍时，新代码的输出与旧代码**逐点完全一致**；
* 场景 3/3b/4 = 复现 `check_num == 0`（`1/0`→Inf→NaN）与路径中含 NaN 的另外两条卡死路线。

### 5.3 静态确认

`dyn_a_star.cpp` 里其余的循环都**有界**，不是本次故障点：

* `:99-104 / :110-115` `do{...}while(checkOccupancy(...))`：`Coord2Index()` 出池即 `return false`，有界；
* `:406-424` `do{...}while(near_flag)`：每次 `near_flag=true` 必然 `erase` 一个点（size 严格递减），且 `size<=2` 时 `break`，有界；
* `:439-453` `do{...}while(too_long_flag && debug_num < 10)`：有硬上限，有界。

## 6. 复跑建议 / 观察点

```bash
source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
cd /home/safina/keyan_project/something1017/TECH/sim_assets
./run_formation.sh 3 --headless          # 或加 --no-gen 跳过 SDF 生成
```

跑完后自查"是否还有 planner 提前静默"：

```bash
R=$(ls -t ~/.ros/log/*/rosout.log | head -1)
# 每个节点的最后一条消息时间：planner 应与其它节点一起跑到最后
grep -a -o '^\([0-9.]*\) [A-Z]* \(/[a-zA-Z0-9_]*\)' $R \
  | awk '{if($1+0>m[$3]) m[$3]=$1+0} END{for(n in m) printf "%-40s %10.3f\n", n, m[n]}' | sort -k2 -n
# 修复后新增/预期的日志（说明"最后一小段没清干净"，但节点没死）
grep -a -o 'A\* simple path: the final segment could not be cleared.*' $R | sort | uniq -c
grep -a -c 'I don.t know why' $R     # 应为 0：旧措辞已被替换
```

预期：即使再撞上障碍，会出现
`A* with esdf check usable=0 ... -> retry without the esdf check` 与
`A* simple path: the final segment could not be cleared (path_size=..., round=...)`
这类 **WARN**，但所有 `ego_planner_node` 会一直活到最后一条日志（不再有节点静默）。
