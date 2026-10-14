#!/usr/bin/env bash
# =============================================================================
#  生成多机编队用的 per-drone Gazebo SDF.
#
#  为什么需要它: PX4 的 iris 模型里 mavlink 端口和 LiDAR 话题名都是写死的,
#  多机时必须每架不同 -> 参见 gen_drone_sdf.py 的说明.
#
#  用法: ./gen_multi_models.sh [数量]      (默认 5)
#  输出: sim_assets/models/iris_lidar_multi/iris_lidar_{0,1,...}.sdf
# =============================================================================
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)
SIM_ASSETS_DIR=$(cd "$SCRIPT_DIR/.." >/dev/null && pwd)

NUM=${1:-5}
PX4_DIR=${PX4_DIR:-/home/safina/PX4-Autopilot}
SITL_GAZEBO="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
OUT_DIR="$SIM_ASSETS_DIR/models/iris_lidar_multi"

if [ ! -f "$SITL_GAZEBO/models/iris/iris.sdf.jinja" ]; then
  echo "[gen_multi_models] 找不到 PX4 的 iris.sdf.jinja, PX4_DIR 对吗? ($PX4_DIR)" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

for ((i = 0; i < NUM; i++)); do
  /usr/bin/python3 "$SCRIPT_DIR/gen_drone_sdf.py" \
    --iris-jinja "$SITL_GAZEBO/models/iris/iris.sdf.jinja" \
    --sitl-gazebo "$SITL_GAZEBO" \
    --mavlink-id "$((1 + i))" \
    --mavlink-udp-port "$((14560 + i))" \
    --mavlink-tcp-port "$((4560 + i))" \
    --lidar-topic "/drone_${i}/points" \
    --imu-topic "/drone_${i}/imu" \
    --output-file "$OUT_DIR/iris_lidar_${i}.sdf"
done

echo "[gen_multi_models] 已生成 $NUM 个机型到 $OUT_DIR"
