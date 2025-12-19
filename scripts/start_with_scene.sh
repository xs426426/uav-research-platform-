#!/bin/bash
# ============================================
# 场景选择启动脚本
# 使用方法: ./start_with_scene.sh
# ============================================

echo "=========================================="
echo "      无人机仿真 - 场景选择启动"
echo "=========================================="
echo ""
echo "可用场景:"
echo "  1. indoor_room    - 室内房间（四面墙+障碍物）⭐推荐"
echo "  2. warehouse      - 仓库（货架场景）"
echo "  3. baylands       - 湾区（户外导航）"
echo "  4. empty          - 空地（基础测试）"
echo "  5. sonoma_raceway - 赛道（围栏环境）"
echo "  6. mcmillan       - 机场（大型户外）"
echo "  7. boat           - 船舶（海上测试）"
echo ""
echo "=========================================="
read -p "选择场景 (1-7，默认1): " choice

case $choice in
    1|"") WORLD="indoor_room" ;;
    2) WORLD="warehouse" ;;
    3) WORLD="baylands" ;;
    4) WORLD="empty" ;;
    5) WORLD="sonoma_raceway" ;;
    6) WORLD="mcmillan_airfield" ;;
    7) WORLD="boat" ;;
    *) WORLD="indoor_room" ;;
esac

echo ""
echo "选择传感器配置:"
echo "  1. 深度相机（默认）"
echo "  2. 2D激光雷达"
echo "  3. 双目相机"
echo "  4. 基础无人机（无传感器）"
echo ""
read -p "选择传感器 (1-4，默认1): " sensor

case $sensor in
    1|"") SDF="iris_depth_camera/iris_depth_camera.sdf" ;;
    2) SDF="iris_rplidar/iris_rplidar.sdf" ;;
    3) SDF="iris_stereo_camera/iris_stereo_camera.sdf" ;;
    4) SDF="" ;;
    *) SDF="iris_depth_camera/iris_depth_camera.sdf" ;;
esac

echo ""
echo "=========================================="
echo "启动配置:"
echo "  场景: $WORLD"
echo "  传感器: ${SDF:-基础无人机}"
echo "=========================================="
echo ""

# 停止已有进程
echo "停止已有仿真进程..."
killall -9 px4 gzserver gzclient 2>/dev/null
sleep 2

# 复制自定义场景到 PX4 目录（如果存在）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
if [ -f "$PROJECT_DIR/worlds/indoor_room.world" ]; then
    cp "$PROJECT_DIR/worlds/indoor_room.world" ~/PX4-Autopilot/Tools/sitl_gazebo/worlds/
    echo "已复制 indoor_room.world 到 PX4 目录"
fi

# 设置环境
cd ~/PX4-Autopilot
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

echo ""
echo "启动仿真..."
echo "启动后请执行: param set COM_RCL_EXCEPT 4"
echo ""

# 启动
if [ -z "$SDF" ]; then
    roslaunch px4 mavros_posix_sitl.launch \
        vehicle:=iris \
        world:=$(pwd)/Tools/sitl_gazebo/worlds/${WORLD}.world
else
    roslaunch px4 mavros_posix_sitl.launch \
        vehicle:=iris \
        sdf:=$(pwd)/Tools/sitl_gazebo/models/${SDF} \
        world:=$(pwd)/Tools/sitl_gazebo/worlds/${WORLD}.world
fi
