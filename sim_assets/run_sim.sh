#!/usr/bin/env bash
# =============================================================================
#  一键跑起整套仿真:
#     PX4 SITL + Gazebo Classic + mavros
#          -> FAST-LIO (占用 Gazebo 的 3D LiDAR + IMU)
#          -> to_mavros/vision2mavros  (FAST-LIO odom -> /mavros/vision_pose/pose)
#          -> ego-planner              (点云 + odom -> pos_cmd)
#          -> to_mavros/poscmd2mavros  (pos_cmd -> /mavros/setpoint_raw/local, OFFBOARD)
#
#  用法:
#      ./run_sim.sh              # 全部启动(带 Gazebo 界面)
#      ./run_sim.sh --headless   # 不带 Gazebo 界面(远程/无显示器时用)
#      ./run_sim.sh --no-px4     # 只启动 FAST-LIO / planner / 桥接 (PX4 已在别的终端跑着)
#      ./run_sim.sh --vision     # (默认已开) 把 FAST-LIO 位姿喂给 /mavros/vision_pose
#      ./run_sim.sh --gps        # 关掉外部视觉, 退回 PX4 自带 GPS/磁力计定位
#                                #   注意: PX4 机型 10015_gazebo-classic_iris 的默认参数
#                                #   已被改成"纯 SLAM 定位"(EKF2_GPS_CTRL=0 / EKF2_EV_CTRL=15
#                                #   / EKF2_HGT_REF=3), 所以**不加 --gps 时必须**一直开着
#                                #   vision2mavros, 否则 EKF2 没有任何水平位置源 ->
#                                #   解锁报 "Preflight Fail: no position estimate"。
#      ./run_sim.sh --manual     # 不开预设航点, 改为在 RViz 里用 "2D Nav Goal" 手动点目标点
#                                #   (等价于给 ego_planner 传 flight_type:=1)
#  停止: Ctrl-C  (脚本会杀掉自己启动的所有进程)
#
#  重要: roslaunch px4 必须带 interactive:=false。否则 PX4 会起交互式 shell(pxh)，
#        在后台/nohup 的 stdin 上会死循环打印 "pxh> " 把日志撑到 GB 级。
# =============================================================================
set -u

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

START_PX4=1
# PX4 机型 10015_gazebo-classic_iris 默认已改成"纯 SLAM 定位"(EKF2_GPS_CTRL=0),
# 所以外部视觉 vision2mavros 默认必须开着; 要退回 PX4 自带 GPS/磁力计定位加 --gps。
START_VISION=1
GUI=true
FLIGHT_TYPE=2          # 2=预设航点  1=RViz 2D Nav Goal 手动指路
for a in "$@"; do
  case "$a" in
    --no-px4)   START_PX4=0 ;;
    --vision)   START_VISION=1 ;;
    --gps|--no-vision) START_VISION=0 ;;
    --headless) GUI=false ;;
    --manual)   FLIGHT_TYPE=1 ;;
    *) echo "未知参数: $a" ;;
  esac
done

# shellcheck disable=SC1090
source "$SCRIPT_DIR/env.sh"

PIDS=()
cleanup() {
  echo ""
  echo "[run_sim] 正在停止所有仿真进程 ..."
  for pid in "${PIDS[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${PIDS[@]}"; do
    kill -9 "$pid" 2>/dev/null || true
  done
  echo "[run_sim] 已退出。日志在 $LOG_DIR"
}
trap cleanup INT TERM EXIT

launch() {          # launch <名字> <roslaunch ...>
  local name=$1; shift
  echo "[run_sim] 启动 $name -> $LOG_DIR/$name.log"
  "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=($!)
}

if [ "$START_PX4" = "1" ]; then
  launch px4_gazebo roslaunch px4 mavros_posix_sitl.launch \
      vehicle:=iris \
      sdf:="$SCRIPT_DIR/models/iris_lidar3d/iris_lidar3d.sdf" \
      world:="$SCRIPT_DIR/worlds/ego_sim.world" \
      gui:=$GUI \
      interactive:=false
  echo "[run_sim] 等 PX4 + Gazebo + mavros 起来 (30s) ..."
  sleep 30

  # ---------------------------------------------------------------------------
  # 关键: 关掉 SITL 的模拟电池掉电
  # ---------------------------------------------------------------------------
  # PX4 的 battery_simulator 逻辑是: 解锁期间电池从 100% 按 SIM_BAT_DRAIN(默认60s)
  # 的速度掉电, 下限 SIM_BAT_MIN_PCT(默认50%); 一上锁就回满。
  # 于是解锁约 30s 后电压掉到 preflight 门槛以下 -> health_and_arming_checks 报
  # "Preflight Fail: Battery unhealthy" -> commander 在 COM_DISARM_PRFLT(10s) 后
  # 自动上锁 -> poscmd2mavros 又去解锁 -> 死循环, 飞机永远飞不起来。
  # 办法: 把最低电量设成 100%, 掉电周期拉到最大。
  echo "[run_sim] 关闭 SITL 模拟电池掉电 (SIM_BAT_MIN_PCT=100) ..."
  # 注意: 不能用 `rosrun mavros mavparam set`, 它会把 FLOAT 参数按 INT 发, PX4 类型
  # 不匹配会拒绝, 报 "OSError: Request failed."。必须用带 value.real 的 rosservice。
  rosservice call /mavros/param/set \
      "{param_id: 'SIM_BAT_MIN_PCT', value: {integer: 0, real: 100.0}}" >/dev/null 2>&1
  rosservice call /mavros/param/set \
      "{param_id: 'SIM_BAT_DRAIN', value: {integer: 0, real: 86400.0}}" >/dev/null 2>&1
fi

# ---------------------------------------------------------------------------
# --gps: 把机型默认改掉的定位参数恢复成"GPS + 磁力计"
#   (机型 10015_gazebo-classic_iris 现在默认 EKF2_GPS_CTRL=0 / EKF2_HGT_REF=3,
#    只给 --gps 而不改回来, EKF2 既没有 GPS 也没有外部视觉 -> 解锁失败)
# ---------------------------------------------------------------------------
if [ "$START_VISION" = "0" ]; then
  echo "[run_sim] --gps: 恢复 PX4 用 GPS+气压+磁力计定位 ..."
  for kv in "EKF2_GPS_CTRL:7" "EKF2_HGT_REF:1" "ATT_EXT_HDG_M:0"; do
    rosservice call /mavros/param/set \
        "{param_id: '${kv%%:*}', value: {integer: ${kv##*:}, real: 0.0}}" >/dev/null 2>&1
  done
fi

launch fast_lio roslaunch fast_lio mapping_sim_gazebo.launch
echo "[run_sim] 等 FAST-LIO 初始化(要攒够 IMU 做重力对齐 + 前 100 帧去零) (20s) ..."
sleep 20

if [ "$START_VISION" = "1" ]; then
  launch vision2mavros roslaunch to_mavros vision2mavros.launch
  echo "[run_sim] 等 vision2mavros 取前 100 帧零点 (10s) ..."
  sleep 10
else
  echo "[run_sim] --gps: 跳过 vision2mavros (退回 PX4 自带 GPS/磁力计 EKF)"
fi

launch ego_planner roslaunch ego_planner sim_gazebo.launch flight_type:=$FLIGHT_TYPE
if [ "$FLIGHT_TYPE" = "1" ]; then
  echo "[run_sim] flight_type=1 (手动模式): 在 RViz 里用工具栏的 2D Nav Goal 点目标点"
else
  echo "[run_sim] flight_type=2: 按预设航点自动飞"
fi
sleep 5

launch poscmd2mavros roslaunch to_mavros poscmd2mavros.launch \
    takeoff_enable:=true takeoff_height:=1.0

echo ""
echo "[run_sim] 全部已启动。RViz 请另开终端:"
echo "          source $SCRIPT_DIR/env.sh && roslaunch ego_planner rviz.launch"
echo "          (若 RViz 固定坐标系不是 world, 改成 world 或 camera_init)"
echo "[run_sim] 观察是否飞起来:"
echo "          rostopic echo /mavros/state"
echo "          rostopic echo /mavros/local_position/pose"
echo "[run_sim] Ctrl-C 结束。日志在 $LOG_DIR"
wait
