#!/usr/bin/env bash
# =============================================================================
#  roslaunch 的 launch-prefix 用: 先等 <秒> 再 exec 真正要跑的命令.
#
#  为什么会用到它?
#  FAST-LIO 需要在 Gazebo 已经起来、IMU/雷达真的有数据之后再启动,
#  否则它会拿"还没对齐/还没稳定"的几帧去做重力对齐和建图, 状态估计会很糟.
#  用 `launch-prefix="$(find sim_utils)/scripts/delayed_start.sh 25"` 就能
#  让这个节点晚 25 秒再起, 而不用把启动拆成好几个 roslaunch 进程.
#
#  exec 保证 PID 不变, 这样 roslaunch/Ctrl-C 能正常把这个节点杀掉.
# =============================================================================
set -e
delay="$1"; shift
sleep "$delay"
# 行缓冲: rosconsole 的 INFO 走 stdout, 重定向到文件时默认是块缓冲, 日志里会
# 只看到 WARN/ERROR(stderr 不带缓冲), INFO 全被吞掉, 排查问题很痛苦.
# PYTHONUNBUFFERED 同理(给 python 节点).
export PYTHONUNBUFFERED=1
exec stdbuf -oL -eL "$@"
