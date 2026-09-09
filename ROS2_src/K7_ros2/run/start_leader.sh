#!/usr/bin/env bash
# K7 多机编队 —— 领导者一键启动（在这块作为 leader 的 K7 板上运行）。
#
# 启动内容：namespace=leader 下的底盘链路（k7_core: 串口+EKF+TF） + twist_mux 仲裁。
# 它的"状态"（/leader/odom_combined）会被从机订阅；leader 自身的速度源需另开，
# 例如：键盘 -> /leader/cmd_vel_key，或手柄 -> /leader/cmd_vel_joy，或 STM32 固件 APF。
#
# 用法：
#   ./run/start_leader.sh
# 或指定 ROS_DOMAIN_ID：
#   ROS_DOMAIN_ID=5 ./run/start_leader.sh
set -e
cd "$(dirname "$0")/.."
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[leader] ROS_DOMAIN_ID=$ROS_DOMAIN_ID, namespace=leader"
ros2 launch k7_bringup k7_core.launch.py namespace:=leader &
P1=$!
ros2 launch k7_bringup k7_twist_mux.launch.py namespace:=leader &
P2=$!
trap 'kill $P1 $P2 2>/dev/null' INT TERM
wait $P1 $P2
