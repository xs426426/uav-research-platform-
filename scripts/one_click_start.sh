#!/bin/bash
# ============================================
# 一键启动无人机仿真 + RViz + 控制
# ============================================

echo "=========================================="
echo "     无人机仿真一键启动"
echo "=========================================="

# 停止所有已有进程
echo "[1/5] 清理旧进程..."
killall -9 px4 gzserver gzclient rosmaster roscore 2>/dev/null
sleep 2

# 检查模型
MODEL_DIR=~/PX4-Autopilot/Tools/sitl_gazebo/models/iris_full_sensor
if [ ! -d "$MODEL_DIR" ]; then
    echo "[!] 创建传感器模型..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    bash "$SCRIPT_DIR/create_full_sensor_model.sh"
fi

# 场景选择
echo ""
echo "选择场景:"
echo "  1. empty            - 空地（推荐首次测试）"
echo "  2. small_warehouse  - 仓库"
echo "  3. hospital         - 医院"
echo ""
read -p "选择 (1-3，默认1): " choice

case $choice in
    1|"") WORLD="empty" ;;
    2) WORLD="small_warehouse" ;;
    3) WORLD="hospital" ;;
    *) WORLD="empty" ;;
esac

echo ""
echo "[2/5] 启动仿真环境: $WORLD"

# 设置环境
cd ~/PX4-Autopilot
source ~/catkin_ws/devel/setup.bash 2>/dev/null
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

# 后台启动仿真
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris \
    sdf:=$(pwd)/Tools/sitl_gazebo/models/iris_full_sensor/iris_full_sensor.sdf \
    world:=$(pwd)/Tools/sitl_gazebo/worlds/${WORLD}.world \
    x:=0 y:=0 z:=0.3 &

SIM_PID=$!
echo "仿真进程PID: $SIM_PID"

# 等待仿真启动
echo "[3/5] 等待仿真启动..."
sleep 15

# 设置PX4参数（通过mavros）
echo "[4/5] 配置飞控参数..."
rosservice call /mavros/param/set "param_id: 'COM_RCL_EXCEPT'
value:
  integer: 4
  real: 0.0" 2>/dev/null

rosservice call /mavros/param/set "param_id: 'COM_ARM_IMU_ACC'
value:
  integer: 0
  real: 10.0" 2>/dev/null

rosservice call /mavros/param/set "param_id: 'CBRK_SUPPLY_CHK'
value:
  integer: 894281
  real: 0.0" 2>/dev/null

# 启动RViz
echo "[5/5] 启动可视化..."
sleep 2

# 创建临时RViz配置
RVIZ_CONFIG="/tmp/uav_config.rviz"
cat > $RVIZ_CONFIG << 'RVIZEOF'
Panels:
  - Class: rviz/Displays
    Name: Displays
Visualization Manager:
  Global Options:
    Fixed Frame: base_link
  Displays:
    - Class: rviz/Grid
      Name: Grid
      Enabled: true
    - Class: rviz/PointCloud2
      Name: 3D LiDAR
      Enabled: true
      Topic: /velodyne_points
      Size (m): 0.05
      Style: Points
      Color Transformer: Intensity
    - Class: rviz/PointCloud2
      Name: Depth Camera
      Enabled: true
      Topic: /iris/camera/depth/points
      Size (m): 0.03
      Style: Points
      Color Transformer: AxisColor
    - Class: rviz/Image
      Name: Camera
      Enabled: true
      Topic: /iris/camera/rgb/image_raw
RVIZEOF

rviz -d $RVIZ_CONFIG &

echo ""
echo "=========================================="
echo "  启动完成！"
echo "=========================================="
echo ""
echo "控制无人机:"
echo "  新终端执行: cd ~/uav-research-platform-/src/phase1_planning && python3 goal_sender.py"
echo "  按 t 起飞 | g 发送目标点 | l 降落"
echo ""
echo "或在pxh>终端执行:"
echo "  commander arm -f"
echo "  commander takeoff"
echo ""
echo "按 Ctrl+C 退出所有进程"
echo "=========================================="

# 等待用户退出
wait $SIM_PID
