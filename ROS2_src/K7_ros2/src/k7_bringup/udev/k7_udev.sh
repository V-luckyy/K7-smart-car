#!/bin/bash
# 在 K7 上执行：安装 udev 规则并使其生效
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo cp "${SCRIPT_DIR}/99-k7-controller.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "完成。重新插拔 STM32 的 USB 线后检查：ls -l /dev/k7_controller"
