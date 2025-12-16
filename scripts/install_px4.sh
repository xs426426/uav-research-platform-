#!/bin/bash
# ============================================================================
# PX4 + Gazebo 安装脚本
# 适用于: Ubuntu 20.04 + ROS Noetic
# ============================================================================

set -e

echo "=========================================="
echo "  PX4 Autopilot + Gazebo 安装脚本"
echo "=========================================="

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_status() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Step 1: 安装依赖
install_dependencies() {
    print_status "安装系统依赖..."

    sudo apt update
    sudo apt install -y \
        git \
        zip \
        cmake \
        build-essential \
        genromfs \
        ninja-build \
        exiftool \
        astyle \
        python3-pip \
        python3-toml \
        python3-numpy \
        python3-dev \
        python3-empy \
        python3-setuptools \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-ugly \
        libeigen3-dev \
        libopencv-dev \
        libxml2-utils

    # Python 依赖
    pip3 install --user \
        jinja2 \
        pyserial \
        cerberus \
        pyulog \
        numpy \
        toml \
        packaging \
        kconfiglib \
        jsonschema
}

# Step 2: 克隆 PX4
clone_px4() {
    print_status "克隆 PX4 Autopilot..."

    PX4_DIR=~/PX4-Autopilot

    if [ -d "$PX4_DIR" ]; then
        print_warning "PX4 目录已存在: $PX4_DIR"
        read -p "是否重新克隆? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf $PX4_DIR
        else
            return
        fi
    fi

    cd ~
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive

    print_status "PX4 克隆完成"
}

# Step 3: 运行 PX4 安装脚本
setup_px4() {
    print_status "运行 PX4 官方安装脚本..."

    cd ~/PX4-Autopilot

    # 使用官方安装脚本
    bash ./Tools/setup/ubuntu.sh --no-sim-tools

    print_status "PX4 依赖安装完成"
}

# Step 4: 安装 Gazebo
install_gazebo() {
    print_status "安装 Gazebo..."

    # ROS Noetic 自带 Gazebo 11，但需要额外的 ROS 集成包
    sudo apt install -y \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control

    print_status "Gazebo 安装完成"
}

# Step 5: 编译 PX4 SITL
build_px4_sitl() {
    print_status "编译 PX4 SITL..."

    cd ~/PX4-Autopilot

    # 首次编译（会下载很多东西，可能需要较长时间）
    DONT_RUN=1 make px4_sitl gazebo

    print_status "PX4 SITL 编译完成"
}

# Step 6: 配置环境变量
setup_px4_environment() {
    print_status "配置 PX4 环境变量..."

    # 添加 PX4 相关环境变量
    if ! grep -q "PX4-Autopilot" ~/.bashrc; then
        cat >> ~/.bashrc << 'EOF'

# PX4 Autopilot
export PX4_HOME=~/PX4-Autopilot
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
EOF
        print_status "PX4 环境变量已添加到 ~/.bashrc"
    else
        print_warning "PX4 环境变量已存在"
    fi
}

# Step 7: 验证安装
verify_installation() {
    print_status "验证安装..."

    # 检查 PX4 目录
    if [ -d ~/PX4-Autopilot ]; then
        print_status "✓ PX4 目录存在"
    else
        print_error "✗ PX4 目录不存在"
        exit 1
    fi

    # 检查 Gazebo
    if command -v gazebo &> /dev/null; then
        print_status "✓ Gazebo 已安装: $(gazebo --version | head -1)"
    else
        print_error "✗ Gazebo 未安装"
        exit 1
    fi
}

# 主函数
main() {
    install_dependencies
    clone_px4
    setup_px4
    install_gazebo
    build_px4_sitl
    setup_px4_environment
    verify_installation

    echo ""
    echo "=========================================="
    echo -e "${GREEN}  PX4 + Gazebo 安装完成!${NC}"
    echo "=========================================="
    echo ""
    echo "请执行以下命令使环境变量生效:"
    echo "  source ~/.bashrc"
    echo ""
    echo "测试仿真:"
    echo "  cd ~/PX4-Autopilot"
    echo "  make px4_sitl gazebo"
    echo ""
    echo "首次启动可能需要下载 Gazebo 模型，请耐心等待..."
    echo ""
}

main
