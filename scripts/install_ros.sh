#!/bin/bash
# ============================================================================
# ROS Noetic 完整安装脚本
# 适用于: Ubuntu 20.04 LTS
# 作者: UAV Research Platform
# ============================================================================

set -e  # 遇到错误立即退出

echo "=========================================="
echo "  ROS Noetic 安装脚本"
echo "  适用于 Ubuntu 20.04"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查Ubuntu版本
check_ubuntu_version() {
    print_status "检查 Ubuntu 版本..."
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [ "$VERSION_ID" != "20.04" ]; then
            print_warning "当前系统: $VERSION_ID，推荐使用 Ubuntu 20.04"
            read -p "是否继续安装? (y/n) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
        else
            print_status "Ubuntu 版本检查通过: $VERSION_ID"
        fi
    fi
}

# Step 1: 设置 sources.list
setup_sources() {
    print_status "配置 ROS 软件源..."
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
}

# Step 2: 设置密钥
setup_keys() {
    print_status "添加 ROS 密钥..."
    sudo apt install -y curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
}

# Step 3: 安装 ROS
install_ros() {
    print_status "更新软件包列表..."
    sudo apt update

    print_status "安装 ROS Noetic Desktop Full..."
    sudo apt install -y ros-noetic-desktop-full

    print_status "安装常用 ROS 包..."
    sudo apt install -y \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-catkin-tools \
        build-essential
}

# Step 4: 初始化 rosdep
init_rosdep() {
    print_status "初始化 rosdep..."
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
}

# Step 5: 配置环境变量
setup_environment() {
    print_status "配置环境变量..."

    # 检查是否已经添加
    if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# ROS Noetic" >> ~/.bashrc
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
        print_status "已添加 ROS 环境变量到 ~/.bashrc"
    else
        print_warning "ROS 环境变量已存在于 ~/.bashrc"
    fi

    # 立即生效
    source /opt/ros/noetic/setup.bash
}

# Step 6: 创建 catkin 工作空间
create_workspace() {
    print_status "创建 catkin 工作空间..."

    WORKSPACE=~/catkin_ws

    if [ -d "$WORKSPACE" ]; then
        print_warning "工作空间 $WORKSPACE 已存在"
    else
        mkdir -p $WORKSPACE/src
        cd $WORKSPACE
        catkin_make

        # 添加工作空间到环境变量
        if ! grep -q "source ~/catkin_ws/devel/setup.bash" ~/.bashrc; then
            echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
        fi

        print_status "工作空间创建完成: $WORKSPACE"
    fi
}

# Step 7: 验证安装
verify_installation() {
    print_status "验证 ROS 安装..."

    source /opt/ros/noetic/setup.bash

    # 检查 roscore
    if command -v roscore &> /dev/null; then
        print_status "✓ roscore 可用"
    else
        print_error "✗ roscore 不可用"
        exit 1
    fi

    # 检查版本
    print_status "ROS 版本: $(rosversion -d)"
}

# 主函数
main() {
    check_ubuntu_version
    setup_sources
    setup_keys
    install_ros
    init_rosdep
    setup_environment
    create_workspace
    verify_installation

    echo ""
    echo "=========================================="
    echo -e "${GREEN}  ROS Noetic 安装完成!${NC}"
    echo "=========================================="
    echo ""
    echo "请执行以下命令使环境变量生效:"
    echo "  source ~/.bashrc"
    echo ""
    echo "测试安装:"
    echo "  roscore  (终端1)"
    echo "  rosrun turtlesim turtlesim_node  (终端2)"
    echo ""
}

# 运行主函数
main
