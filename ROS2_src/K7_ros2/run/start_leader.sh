#!/usr/bin/env bash
# K7 convoy leader: publish raw STM32 state without sending commands back.
set -e
cd "$(dirname "$0")/.."
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[leader] ROS_DOMAIN_ID=$ROS_DOMAIN_ID, namespace=leader, STM32 autonomous"
exec ros2 launch k7_bringup k7_core.launch.py \
  namespace:=leader serial_only:=true enable_downlink:=false
