#!/bin/bash
# K7_ros2 环境一键安装脚本
# 在 K7（Ubuntu 24.04, ARM64）上执行: ./setup_env.sh
# 提示: 网络慢可先把 apt 源换成国内镜像（如清华源）再运行本脚本
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ROS apt 源：默认 USTC 镜像（国内速度快）；想用官方源改为 http://packages.ros.org/ros2/ubuntu
ROS_APT_SRC="https://mirrors.ustc.edu.cn/ros2/ubuntu"

# ========== 1. 安装 ROS2 Jazzy（已装则跳过） ==========
if [ -f /opt/ros/jazzy/setup.bash ]; then
  echo "[1/3] ROS2 Jazzy 已安装，跳过"
else
  echo "[1/3] 安装 ROS2 Jazzy ..."
  sudo apt update
  sudo apt install -y software-properties-common curl
  sudo add-apt-repository -y universe
  # GPG 密钥：优先用仓库自带的（raw.githubusercontent.com 国内不可达会卡死）；文件缺失时才联网下载
  if [ -f "${SCRIPT_DIR}/setup/ros.key" ]; then
    echo "使用本地 setup/ros.key（官方指纹 F42ED6FBAB17C654）"
    sudo cp "${SCRIPT_DIR}/setup/ros.key" /usr/share/keyrings/ros-archive-keyring.gpg
  else
    sudo curl -sSL --connect-timeout 15 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
  fi
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_APT_SRC} $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
  sudo apt install -y ros-jazzy-desktop ros-dev-tools
fi

# ========== 2. 安装项目依赖 ==========
echo "[2/3] 安装项目依赖 ..."
sudo apt install -y \
  python3-colcon-common-extensions \
  v4l-utils \
  ros-jazzy-usb-cam \
  ros-jazzy-stereo-image-proc \
  ros-jazzy-camera-calibration \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-twist-mux \
  ros-jazzy-robot-localization \
  ros-jazzy-imu-filter-madgwick \
  ros-jazzy-depthimage-to-laserscan \
  ros-jazzy-web-video-server \
  ros-jazzy-navigation2

# ========== 3. bashrc 自动 source ROS 环境 ==========
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  echo "[3/3] 已添加 'source /opt/ros/jazzy/setup.bash' 到 ~/.bashrc"
else
  echo "[3/3] ~/.bashrc 已配置，跳过"
fi

echo ""
echo "===== 安装完成 ====="
echo "下一步："
echo "  source ~/.bashrc"
echo "  cd ~/code/K7_ros2 && colcon build --symlink-install"
