#!/usr/bin/env bash
# =============================================================================
#  PX4 SITL + Gazebo Classic + FAST-LIO + ego-planner + TO_MAVROS 仿真环境
#  用法（每个新终端都要 source 一次）:
#       source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
# =============================================================================

PX4_DIR=${PX4_DIR:-/home/safina/PX4-Autopilot}
TECH_WS=${TECH_WS:-/home/safina/keyan_project/something1017/TECH/src/ego-planner-swarm}
# export 出去: 多机编队的 launch 文件里用 $(env SIM_ASSETS_DIR) 定位
# 生成出来的 per-drone SDF / world, 不 export 的话 roslaunch 取不到。
export SIM_ASSETS_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)
SITL_GAZEBO_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

# ---------------------------------------------------------------------------
# 关键: 保护调用者的 `set -u`
# ---------------------------------------------------------------------------
# 下面要 source 的 PX4 `setup_gazebo.bash` 里有这样的写法:
#     export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${BUILD_DIR}/build_gazebo-classic
# 它引用的 GAZEBO_PLUGIN_PATH / GAZEBO_MODEL_PATH / LD_LIBRARY_PATH 很可能"未定义"
# (ROS 自己并不设置 GAZEBO_PLUGIN_PATH)。如果调用者(比如 run_sim.sh)开了 `set -u`,
# bash 会因此**直接终止整个脚本**, 而调用者往往把 source 的输出重定向掉了
# (source env.sh >/dev/null 2>&1) -> 表现就是"脚本什么都没干就结束了", 极难排查。
# 办法: 先备份调用者的 shell 选项, 本文件内部临时 set +u, 结束前恢复。
_ENV_SH_SAVED_OPTS=$(set +o)
set +u

# ---------------------------------------------------------------------------
# 关键: 让 `python3` 指向系统 python3.8, 而不是 miniconda 的 python3.13
# ---------------------------------------------------------------------------
# ROS Noetic 里不少脚本(gazebo_ros/spawn_model, gazebo_ros/gazebo 等)的 shebang
# 是 `#!/usr/bin/env python3`, 会跟着 PATH 走。若 PATH 里 miniconda 在前,
# 就会用 3.13 解释器跑 ROS 的 python2/3.8 代码, 于是:
#     ModuleNotFoundError: No module named 'netifaces'
#     (rosgraph.network.get_local_addresses 里 import netifaces)
# 结果是 spawn_model 崩掉 -> Gazebo 里根本没有无人机 -> /drone_0/* 一个话题都没有。
# 这里只把 /usr/bin 加到 PATH 最前, 不动 conda 本身; 需要 conda 时显式用
# `conda activate` 或绝对路径即可。
if [ "$(command -v python3)" != "/usr/bin/python3" ]; then
  export PATH="/usr/bin:$PATH"
  echo "[env.sh] 已把 /usr/bin 提到 PATH 最前 (ROS 需要 python3.8): python3 -> $(command -v python3)"
fi

# --- ROS1 ---
source /opt/ros/noetic/setup.bash

# --- 本仓库的工作区（ego_planner / quadrotor_msgs / fast_lio / to_mavros ...）---
if [ -f "$TECH_WS/devel/setup.bash" ]; then
  source "$TECH_WS/devel/setup.bash"
else
  echo "[env.sh] 警告: 未找到 $TECH_WS/devel/setup.bash，请先编译该工作区" >&2
fi

# --- PX4 + Gazebo Classic（插件路径 / 模型路径 / 库路径）---
# 双保险: PX4 的 setup_gazebo.bash 会做 `VAR=$VAR:xxx`, 这里先补上默认值
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
if [ -f "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" ]; then
  # shellcheck disable=SC1090
  source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
         "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"
else
  echo "[env.sh] 警告: 未找到 PX4 的 setup_gazebo.bash" >&2
fi

# --- 本仓库自己的 Gazebo 模型 / 世界 ---
export GAZEBO_MODEL_PATH="$SIM_ASSETS_DIR/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="$SIM_ASSETS_DIR/worlds:${GAZEBO_RESOURCE_PATH}"

# --- 让 roslaunch 能 $(find px4) / $(find mavlink_sitl_gazebo) ---
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${PX4_DIR}:${SITL_GAZEBO_DIR}:${SIM_ASSETS_DIR}"

# --- 单机仿真用 localhost，避免 ROS_HOSTNAME 解析失败 ---
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311/}

# --- PX4 SITL 机型（与 roslaunch px4 里 vehicle:=iris 配套）---
export PX4_SIM_MODEL=gazebo-classic_iris

echo "[env.sh] PX4_DIR          = $PX4_DIR"
echo "[env.sh] TECH_WS          = $TECH_WS"
echo "[env.sh] SIM_ASSETS_DIR   = $SIM_ASSETS_DIR"
echo "[env.sh] GAZEBO_MODEL_PATH = $GAZEBO_MODEL_PATH"
echo "[env.sh] ROS_PACKAGE_PATH  = $ROS_PACKAGE_PATH"
echo "[env.sh] GAZEBO_PLUGIN_PATH= ${GAZEBO_PLUGIN_PATH:-<空>}"

# ---------------------------------------------------------------------------
# 恢复调用者原来的 shell 选项(把之前为保护 `set -u` 而 set +u 的影响撤掉)
# 注意: 这一步必须放在文件最后 —— 后面不要再加任何语句。
# ---------------------------------------------------------------------------
if [ -n "${_ENV_SH_SAVED_OPTS:-}" ]; then
  eval "$_ENV_SH_SAVED_OPTS"
  unset _ENV_SH_SAVED_OPTS
fi
