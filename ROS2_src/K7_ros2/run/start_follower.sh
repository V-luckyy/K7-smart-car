#!/usr/bin/env bash
# K7 多机编队 —— 跟随者一键启动（在每个作为 follower 的 K7 板上运行）。
#
# 启动内容：namespace=followerN 下的底盘链路(k7_core) + twist_mux + 编队跟随节点。
# follower_node 订阅 /leader/odom_combined，算出 cmd_vel_follower -> twist_mux -> 底盘。
#
# 用法：
#   ./run/start_follower.sh follower1          # 1 号从机，间距默认 1.2 m
#   ./run/start_follower.sh follower2 1.5      # 2 号从机，间距 1.5 m
# 或指定 ROS_DOMAIN_ID：
#   ROS_DOMAIN_ID=5 ./run/start_follower.sh follower1
set -e
cd "$(dirname "$0")/.."
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

NS="${1:-follower1}"
DIST="${2:-1.2}"

echo "[follower] ROS_DOMAIN_ID=$ROS_DOMAIN_ID, namespace=$NS, distance=$DIST m"
exec ros2 launch k7_follower k7_follower.launch.py namespace:="$NS" distance:="$DIST"
