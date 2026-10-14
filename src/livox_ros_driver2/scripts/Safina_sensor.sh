#!/bin/bash
set -e

PIDS=()

cleanup(){
        for pid in "${PIDS[@]}"; do
                if kill -0 "$pid" 2>/dev/null; then
                        kill "$pid"
                fi
        done
        wait
}

trap cleanup TERM INT HUP

source "/opt/ros/noetic/setup.bash"
source "/home/orangepi/something1017/TECH/devel/setup.bash"

# source "~/.bashrc"
export ROS_HOSTNAME=orangepi5ultra
export ROS_MASTER_URI=http://localhost:11311/

sleep 3
roslaunch mavros px4.launch &
PIDS+=($!)


sleep 3
rm -f /dev/shm/orbbec_device.lock
roslaunch orbbec_camera gemini_330_series.launch &
PIDS+=($!)

sleep 3
roslaunch usb_cam usb_cam.launch &
PIDS+=($!)


sleep 25
roslaunch livox_ros_driver2 msg_MID360s.launch &
PIDS+=($!)



for pid in "${PIDS[@]}"; do
        wait "$pid" || {
                exit 1
        }
done
