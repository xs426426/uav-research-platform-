#!/bin/bash
# ============================================================================
# ROS Noetic 安装脚本 - 中国镜像版
# 适用于: Ubuntu 20.04 LTS
# ============================================================================

set -e

echo "=========================================="
echo "  ROS Noetic 安装脚本 (中国镜像)"
echo "  适用于 Ubuntu 20.04"
echo "=========================================="

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_status() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }

# Step 1: 使用清华镜像源
setup_sources() {
    print_status "配置 ROS 软件源 (清华镜像)..."
    sudo sh -c 'echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
}

# Step 2: 添加密钥 (使用国内可访问的方式)
setup_keys() {
    print_status "添加 ROS 密钥..."

    # 方法1: 使用 apt-key 直接添加
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

    print_status "密钥添加成功"
}

# Step 3: 安装 ROS
install_ros() {
    print_status "更新软件包列表..."
    sudo apt update

    print_status "安装 ROS Noetic Desktop Full (可能需要 10-20 分钟)..."
    sudo apt install -y ros-noetic-desktop-full

    print_status "安装开发工具..."
    sudo apt install -y \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-catkin-tools \
        build-essential
}

# Step 4: 初始化 rosdep (使用国内源)
init_rosdep() {
    print_status "初始化 rosdep..."

    # 如果已经初始化过，跳过
    if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        print_warning "rosdep 已初始化，跳过"
    else
        sudo rosdep init || print_warning "rosdep init 失败，尝试继续..."
    fi

    # 使用 rosdistro 国内镜像
    print_status "配置 rosdep 国内镜像..."

    # 修改 rosdep 源
    export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml

    rosdep update || print_warning "rosdep update 失败，可稍后手动执行"
}

# Step 5: 配置环境变量
setup_environment() {
    print_status "配置环境变量..."

    if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# ROS Noetic" >> ~/.bashrc
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
        print_status "已添加 ROS 环境变量到 ~/.bashrc"
    else
        print_warning "ROS 环境变量已存在"
    fi

    source /opt/ros/noetic/setup.bash
}

# Step 6: 创建 catkin 工作空间
create_workspace() {
    print_status "创建 catkin 工作空间..."

    WORKSPACE=~/catkin_ws

    if [ -d "$WORKSPACE" ]; then
        print_warning "工作空间已存在: $WORKSPACE"
    else
        mkdir -p $WORKSPACE/src
        cd $WORKSPACE
        source /opt/ros/noetic/setup.bash
        catkin_make

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

    if command -v roscore &> /dev/null; then
        print_status "✓ roscore 可用"
        print_status "✓ ROS 版本: $(rosversion -d)"
    else
        echo "roscore 不可用，安装可能失败"
        exit 1
    fi
}

main() {
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
    echo "请执行: source ~/.bashrc"
    echo ""
    echo "测试: roscore"
    echo ""
}

main
