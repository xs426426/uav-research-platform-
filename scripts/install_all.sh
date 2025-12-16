#!/bin/bash
# ============================================================================
# 一键安装所有环境
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "  无人机仿真平台 - 完整环境安装"
echo "=========================================="
echo ""
echo "将安装以下组件:"
echo "  1. ROS Noetic"
echo "  2. PX4 Autopilot + Gazebo"
echo "  3. MAVROS"
echo ""
read -p "是否继续? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 0
fi

# 安装 ROS
echo ""
echo "========== Step 1/3: 安装 ROS Noetic =========="
bash "$SCRIPT_DIR/install_ros.sh"

# 重新加载环境变量
source ~/.bashrc

# 安装 PX4 + Gazebo
echo ""
echo "========== Step 2/3: 安装 PX4 + Gazebo =========="
bash "$SCRIPT_DIR/install_px4.sh"

# 重新加载环境变量
source ~/.bashrc

# 安装 MAVROS
echo ""
echo "========== Step 3/3: 安装 MAVROS =========="
bash "$SCRIPT_DIR/install_mavros.sh"

echo ""
echo "=========================================="
echo "  所有组件安装完成!"
echo "=========================================="
echo ""
echo "请执行以下命令重新加载环境:"
echo "  source ~/.bashrc"
echo ""
echo "然后可以测试仿真:"
echo "  # 终端1: 启动 PX4 SITL + Gazebo"
echo "  cd ~/PX4-Autopilot && make px4_sitl gazebo"
echo ""
echo "  # 终端2: 启动 MAVROS"
echo "  roslaunch mavros px4.launch fcu_url:=\"udp://:14540@127.0.0.1:14557\""
echo ""
echo "  # 终端3: 运行控制脚本"
echo "  python3 ~/catkin_ws/src/uav_control/scripts/offboard_control.py"
echo ""
