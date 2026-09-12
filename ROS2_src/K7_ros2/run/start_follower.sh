#!/usr/bin/env bash
# K7 convoy follower: serial bridge + twist_mux + lightweight trail follower.
# Usage:
#   ./run/start_follower.sh follower1 1.2 /leader/odom
#   ./run/start_follower.sh follower2 1.2 /follower1/odom
set -e
cd "$(dirname "$0")/.."
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

NS="${1:-follower1}"
DIST="${2:-1.2}"
LEADER_TOPIC="${3:-/leader/odom}"

echo "[follower] ROS_DOMAIN_ID=$ROS_DOMAIN_ID, namespace=$NS, distance=$DIST m, leader=$LEADER_TOPIC"
exec ros2 launch k7_follower k7_follower.launch.py \
  namespace:="$NS" distance:="$DIST" leader_topic:="$LEADER_TOPIC"
