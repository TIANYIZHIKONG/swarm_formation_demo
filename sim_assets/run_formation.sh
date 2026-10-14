#!/usr/bin/env bash
# =============================================================================
#  多机编队仿真一键启动
#    Gazebo + PX4 SITL x N + mavros x N + FAST-LIO x N + Swarm-Formation x N
#    + vision2mavros x N (FAST-LIO -> PX4 外部视觉; PX4 侧已是纯 SLAM 定位)
#    + swarm_bridge (机间轨迹交换)
#
#  用法 (每个新终端先 source 一次 env.sh):
#      source /home/safina/keyan_project/something1017/TECH/sim_assets/env.sh
#      ./run_formation.sh 3              # 3 架, 带 Gazebo GUI
#      ./run_formation.sh 3 --headless   # 无 GUI
#      ./run_formation.sh 3 --no-gen     # 不重新生成 per-drone SDF
#
#  然后另开终端:
#      roslaunch sim_utils formation_rviz.launch
#    在 RViz 工具栏点 "2D Nav Goal", 在空地上点一个"编队中心"目标 ->
#    所有无人机各自飞到 relative_pos_<id> 阵位上, 保持队形一起飞.
#
#  停止: Ctrl-C (脚本会杀掉自己启动的所有进程)
# =============================================================================
set -u

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

NUM=3
GUI=true
GEN_SDF=true
for a in "$@"; do
  case "$a" in
    --headless) GUI=false ;;
    --no-gen)   GEN_SDF=false ;;
    ''|*[!0-9]*) echo "[run_formation] 未知参数: $a" ;;
    *) NUM=$a ;;
  esac
done

if [ "$NUM" -lt 1 ] || [ "$NUM" -gt 5 ]; then
  echo "[run_formation] 架数只支持 1..5 (受 launch 里手写的 drone 块限制)" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$SCRIPT_DIR/env.sh"

# ---------------------------------------------------------------------------
# 1) 生成 per-drone 的 Gazebo SDF
#    型号文件里包含每机不同的 mavlink 端口 (14560+i / 4560+i) 和 LiDAR/IMU
#    话题名 (/drone_<i>/points, /drone_<i>/imu). 详见 scripts/gen_drone_sdf.py.
#    固定生成 5 架, 这样改 num_drones 不用重新生成.
# ---------------------------------------------------------------------------
if [ "$GEN_SDF" = "true" ]; then
  "$SCRIPT_DIR/scripts/gen_multi_models.sh" 5 || {
    echo "[run_formation] 生成机型失败, 退出" >&2; exit 1; }
fi

PIDS=()
cleanup() {
  echo ""
  echo "[run_formation] 正在停止所有仿真进程 ..."
  for pid in "${PIDS[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 3
  for pid in "${PIDS[@]}"; do
    kill -9 "$pid" 2>/dev/null || true
  done
  echo "[run_formation] 已退出。日志在 $LOG_DIR"
}
trap cleanup INT TERM EXIT

# ---------------------------------------------------------------------------
# 2) 关掉 SITL 的"模拟电池掉电" (每架都要设)
#    PX4 的 battery_simulator 在解锁后按 SIM_BAT_DRAIN(默认 60s) 从 100% 掉到
#    SIM_BAT_MIN_PCT(默认 50%); 掉到 preflight 门槛以下后 commander 会
#    "Preflight Fail: Battery unhealthy" -> COM_DISARM_PRFLT(10s) 后自动上锁,
#    于是 poscmd2mavros 又去解锁 -> 死循环, 飞机永远飞不起来.
#    注意: 必须用带 value.real 的 rosservice; `rosrun mavros mavparam set`
#    会把 FLOAT 当 INT 发, PX4 类型不匹配直接拒绝.
#    这里在后台轮询 mavros 出现就立刻设, 尽量赶在自动上锁之前.
# ---------------------------------------------------------------------------
set_battery_params() {
  local i
  for ((i = 0; i < NUM; i++)); do
    # 等这一架的 param 服务出现 (最多等 120s)
    local waited=0
    until rosservice list 2>/dev/null | grep -q "\\/drone_${i}\\/mavros\\/param\\/set"; do
      sleep 1
      waited=$((waited + 1))
      [ "$waited" -ge 120 ] && break
    done
    rosservice call "/drone_${i}/mavros/param/set" \
      "{param_id: 'SIM_BAT_MIN_PCT', value: {integer: 0, real: 100.0}}" >/dev/null 2>&1
    rosservice call "/drone_${i}/mavros/param/set" \
      "{param_id: 'SIM_BAT_DRAIN', value: {integer: 0, real: 86400.0}}" >/dev/null 2>&1
    echo "[run_formation] drone_${i}: 已关闭模拟电池掉电"
  done
}
set_battery_params >"$LOG_DIR/battery_fix.log" 2>&1 &
PIDS+=($!)

# ---------------------------------------------------------------------------
# 3) 起整个仿真
#    formation.launch 里 FAST-LIO 自己带了 25s 延迟 (launch-prefix), 等
#    Gazebo + PX4 真的把 IMU/雷达数据发出来再做重力对齐/建图.
# ---------------------------------------------------------------------------
echo "[run_formation] 启动 $NUM 架: Gazebo(含 PX4 SITL) + FAST-LIO + Swarm-Formation"
echo "[run_formation] GUI=$GUI  日志 -> $LOG_DIR/formation.log"
roslaunch sim_utils formation.launch num_drones:="$NUM" gui:="$GUI" \
    >"$LOG_DIR/formation.log" 2>&1 &
PIDS+=($!)

echo ""
echo "[run_formation] 起来要 40~60s (N 架 PX4 SITL + N 个 FAST-LIO 各自初始化)."
echo "[run_formation] 另开一个终端看 RViz:"
echo "        source $SCRIPT_DIR/env.sh"
echo "        roslaunch sim_utils formation_rviz.launch"
echo "[run_formation] 检查每架是否就绪:"
for ((i = 0; i < 1; i++)); do
  echo "        rostopic echo /drone_${i}/mavros/state          # armed / mode"
done
echo "        rostopic hz /drone_0/cloud_registered           # FAST-LIO 出点云"
echo "        rostopic echo /drone_0_visual_slam/odom         # 规划器用的世界系位姿"
echo "        rostopic hz   /drone_0/mavros/vision_pose/pose  # FAST-LIO -> PX4 外部视觉"
echo "        rostopic echo /drone_0/mavros/local_position/pose  # EKF2(纯视觉)定位结果"
echo "[run_formation] 然后在 RViz 里点 '2D Nav Goal' 给一个编队中心点。"
echo "[run_formation] 观察编队: 检查 /drone_<i>_planning/pos_cmd 都有输出。"
echo "[run_formation] Ctrl-C 结束。日志在 $LOG_DIR"
wait
