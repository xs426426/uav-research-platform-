#!/bin/bash
# 快速启动 - 只启动 PX4 + Gazebo + 深度相机
# 使用: ./quick_start.sh

cd ~/PX4-Autopilot
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

echo "启动 PX4 + Gazebo (带深度相机)..."
echo ""
echo "启动后请执行: param set COM_RCL_EXCEPT 4"
echo ""

roslaunch px4 mavros_posix_sitl.launch vehicle:=iris sdf:=$(pwd)/Tools/sitl_gazebo/models/iris_depth_camera/iris_depth_camera.sdf
