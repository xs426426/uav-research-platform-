#!/bin/bash
# ============================================================================
# MAVROS 安装脚本
# 适用于: Ubuntu 20.04 + ROS Noetic
# ============================================================================

set -e

echo "=========================================="
echo "  MAVROS 安装脚本"
echo "=========================================="

GREEN='\033[0;32m'
NC='\033[0m'

print_status() { echo -e "${GREEN}[INFO]${NC} $1"; }

# 安装 MAVROS
install_mavros() {
    print_status "安装 MAVROS..."

    sudo apt update
    sudo apt install -y \
        ros-noetic-mavros \
        ros-noetic-mavros-extras \
        ros-noetic-mavros-msgs

    print_status "MAVROS 安装完成"
}

# 安装 GeographicLib 数据集
install_geographiclib() {
    print_status "安装 GeographicLib 数据集..."

    # 这是 MAVROS 需要的地理数据
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    chmod +x install_geographiclib_datasets.sh
    sudo ./install_geographiclib_datasets.sh
    rm install_geographiclib_datasets.sh

    print_status "GeographicLib 数据集安装完成"
}

# 验证安装
verify_mavros() {
    print_status "验证 MAVROS 安装..."

    source /opt/ros/noetic/setup.bash

    if rospack find mavros &> /dev/null; then
        print_status "✓ MAVROS 安装成功"
        print_status "  路径: $(rospack find mavros)"
    else
        echo "MAVROS 安装失败"
        exit 1
    fi
}

main() {
    install_mavros
    install_geographiclib
    verify_mavros

    echo ""
    echo "=========================================="
    echo -e "${GREEN}  MAVROS 安装完成!${NC}"
    echo "=========================================="
    echo ""
    echo "启动方式:"
    echo "  roslaunch mavros px4.launch fcu_url:=\"udp://:14540@127.0.0.1:14557\""
    echo ""
}

main
