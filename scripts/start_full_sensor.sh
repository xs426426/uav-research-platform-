#!/bin/bash
# ============================================
# 启动带完整传感器的无人机仿真
# 传感器: 深度相机 + 3D激光雷达
# ============================================

echo "=========================================="
echo "  无人机仿真 - 完整传感器版"
echo "  深度相机 + 3D激光雷达(16线)"
echo "=========================================="
echo ""

# 检查模型是否存在
MODEL_DIR=~/PX4-Autopilot/Tools/sitl_gazebo/models/iris_full_sensor
if [ ! -d "$MODEL_DIR" ]; then
    echo "模型不存在，正在创建..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    bash "$SCRIPT_DIR/create_full_sensor_model.sh"
fi

echo "可用场景:"
echo "  1. hospital         - 医院（室内探索推荐）⭐"
echo "  2. small_warehouse  - 仓库（避障测试）"
echo "  3. bookstore        - 书店（SLAM测试）"
echo "  4. empty            - 空地（基础测试）"
echo ""
read -p "选择场景 (1-4，默认1): " choice

case $choice in
    1|"") WORLD="hospital"; X=0; Y=-5; Z=0 ;;
    2) WORLD="small_warehouse"; X=0; Y=0; Z=0 ;;
    3) WORLD="bookstore"; X=0; Y=-3; Z=0 ;;
    4) WORLD="empty"; X=0; Y=0; Z=0 ;;
    *) WORLD="hospital"; X=0; Y=-5; Z=0 ;;
esac

echo ""
echo "启动配置:"
echo "  场景: $WORLD"
echo "  起始位置: X=$X, Y=$Y, Z=$Z"
echo "  传感器: 深度相机 + 3D激光雷达"
echo ""

# 停止已有进程
echo "停止已有仿真进程..."
killall -9 px4 gzserver gzclient 2>/dev/null
sleep 2

# 设置环境
cd ~/PX4-Autopilot
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

echo ""
echo "启动仿真..."
echo ""
echo "=========================================="
echo "启动后的操作步骤:"
echo "  1. 等待 pxh> 提示符出现"
echo "  2. 执行: param set COM_RCL_EXCEPT 4"
echo "  3. 新终端验证传感器:"
echo "     rostopic list | grep -E 'camera|velodyne'"
echo "  4. 启动RViz查看点云:"
echo "     rviz"
echo "=========================================="
echo ""

roslaunch px4 mavros_posix_sitl.launch vehicle:=iris \
    sdf:=$(pwd)/Tools/sitl_gazebo/models/iris_full_sensor/iris_full_sensor.sdf \
    world:=$(pwd)/Tools/sitl_gazebo/worlds/${WORLD}.world \
    x:=$X y:=$Y z:=$Z
