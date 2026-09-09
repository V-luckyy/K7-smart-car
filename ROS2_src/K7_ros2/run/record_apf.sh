#!/usr/bin/env bash
# K7 —— APF+Stanley 实车调参：一键记录（listen-only 解析 + CSV 记录）。
#
# 前置：
#   1) STM32 已烧含 0xFB 调试帧的固件，上电即自跑 APF+Stanley 圆（无需 ROS 控制）；
#   2) 本板 K7 用 USB 线连着该底盘。
#
# 用法：
#   ./run/record_apf.sh                       # 默认参考=line（当前固件走直线）
#   ./run/record_apf.sh ~/apf_log circle      # 固件改回参考圆时传 circle
set -e
cd "$(dirname "$0")/.."
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

OUT="${1:-$(echo ~)/apf_log}"
REF="${2:-line}"
echo "[record] 纯记录模式启动 -> CSV: $OUT/  (reference=$REF)"
exec ros2 launch k7_apf_debug record.launch.py out_dir:="$OUT" reference:="$REF"
