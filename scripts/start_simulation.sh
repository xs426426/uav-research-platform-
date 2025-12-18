#!/bin/bash
# ============================================
# 无人机仿真一键启动脚本
# 使用方法: ./start_simulation.sh
# ============================================

echo "=========================================="
echo "   无人机仿真环境一键启动"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 配置
PX4_DIR=~/PX4-Autopilot
CATKIN_WS=~/catkin_ws
PROJECT_DIR=~/uav-research-platform-

# 检查 PX4 目录
if [ ! -d "$PX4_DIR" ]; then
    echo -e "${RED}错误: PX4 目录不存在 ($PX4_DIR)${NC}"
    exit 1
fi

echo -e "${GREEN}[1/4] 启动 PX4 + Gazebo (带深度相机)...${NC}"

# 终端1: PX4 + Gazebo
gnome-terminal --title="PX4 + Gazebo" -- bash -c "
    cd $PX4_DIR
    source $CATKIN_WS/devel/setup.bash
    source Tools/setup_gazebo.bash \$(pwd) \$(pwd)/build/px4_sitl_default
    export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$(pwd):\$(pwd)/Tools/sitl_gazebo

    echo '正在启动 PX4 + Gazebo...'
    echo '启动后请执行: param set COM_RCL_EXCEPT 4'
    echo ''

    roslaunch px4 mavros_posix_sitl.launch vehicle:=iris sdf:=\$(pwd)/Tools/sitl_gazebo/models/iris_depth_camera/iris_depth_camera.sdf

    exec bash
" &

echo -e "${YELLOW}等待 Gazebo 启动 (20秒)...${NC}"
sleep 20

echo -e "${GREEN}[2/4] 启动 RViz 可视化...${NC}"

# 终端2: RViz
gnome-terminal --title="RViz" -- bash -c "
    source /opt/ros/noetic/setup.bash
    source $CATKIN_WS/devel/setup.bash

    echo '启动 RViz...'
    echo '提示: 添加 PointCloud2 话题 /camera/depth/points'
    echo '      添加 Image 话题 /camera/rgb/image_raw'
    echo ''

    rviz

    exec bash
" &

sleep 3

echo -e "${GREEN}[3/4] 启动传感器监控...${NC}"

# 终端3: 传感器监控
gnome-terminal --title="传感器监控" -- bash -c "
    source /opt/ros/noetic/setup.bash
    source $CATKIN_WS/devel/setup.bash

    echo '========== 传感器话题监控 =========='
    echo ''
    echo '深度相机话题:'
    rostopic list | grep camera
    echo ''
    echo '检查数据频率:'
    echo '按 Ctrl+C 停止后可以手动检查其他话题'
    echo ''

    rostopic hz /camera/depth/image_raw

    exec bash
" &

sleep 2

echo -e "${GREEN}[4/4] 启动无人机控制面板...${NC}"

# 终端4: 控制脚本
gnome-terminal --title="无人机控制" -- bash -c "
    source /opt/ros/noetic/setup.bash
    source $CATKIN_WS/devel/setup.bash

    cd $PROJECT_DIR/src/phase1_planning

    echo '========== 无人机控制面板 =========='
    echo ''
    echo '等待连接...'
    sleep 5

    python3 goal_sender.py

    exec bash
" &

echo ""
echo -e "${GREEN}=========================================="
echo "   所有窗口已启动!"
echo "==========================================${NC}"
echo ""
echo "操作步骤:"
echo "  1. 在 'PX4 + Gazebo' 窗口等待 pxh> 出现"
echo "  2. 执行: param set COM_RCL_EXCEPT 4"
echo "  3. 在 '无人机控制' 窗口按 't' 起飞"
echo "  4. 在 RViz 中观察传感器数据"
echo ""
echo -e "${YELLOW}关闭所有窗口: 按 Ctrl+C 或关闭终端${NC}"
