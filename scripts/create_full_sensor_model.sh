#!/bin/bash
# ============================================
# 创建带3D激光雷达+深度相机的无人机模型
# ============================================

echo "=========================================="
echo "  创建 iris_full_sensor 无人机模型"
echo "  包含: 深度相机 + 3D激光雷达"
echo "=========================================="

# 安装依赖
echo ""
echo "[1/4] 安装Velodyne激光雷达插件..."
sudo apt-get update
sudo apt-get install -y ros-noetic-velodyne-gazebo-plugins \
    ros-noetic-velodyne-description \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions

# 创建模型目录
echo ""
echo "[2/4] 创建模型目录..."
MODEL_DIR=~/PX4-Autopilot/Tools/sitl_gazebo/models/iris_full_sensor
mkdir -p $MODEL_DIR

# 创建model.config
echo ""
echo "[3/4] 创建model.config..."
cat > $MODEL_DIR/model.config << 'EOF'
<?xml version="1.0"?>
<model>
  <name>iris_full_sensor</name>
  <version>1.0</version>
  <sdf version="1.5">iris_full_sensor.sdf</sdf>
  <author>
    <name>UAV Research</name>
  </author>
  <description>
    Iris quadrotor with depth camera and 3D LiDAR
  </description>
</model>
EOF

# 创建SDF文件
echo ""
echo "[4/4] 创建iris_full_sensor.sdf..."
cat > $MODEL_DIR/iris_full_sensor.sdf << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="iris_full_sensor">

    <!-- 包含基础iris无人机 -->
    <include>
      <uri>model://iris</uri>
    </include>

    <!-- ========== 深度相机 ========== -->
    <link name="camera_link">
      <pose>0.1 0 0 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.00001</ixx>
          <iyy>0.00001</iyy>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <visual name="camera_visual">
        <geometry>
          <box><size>0.03 0.08 0.03</size></box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
        </material>
      </visual>
      <sensor name="depth_camera" type="depth">
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
        </camera>
        <plugin name="depth_camera_plugin" filename="libgazebo_ros_openni_kinect.so">
          <robotNamespace></robotNamespace>
          <baseline>0.1</baseline>
          <alwaysOn>true</alwaysOn>
          <updateRate>30.0</updateRate>
          <cameraName>camera</cameraName>
          <imageTopicName>rgb/image_raw</imageTopicName>
          <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
          <depthImageTopicName>depth/image_raw</depthImageTopicName>
          <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
          <pointCloudTopicName>depth/points</pointCloudTopicName>
          <frameName>camera_link</frameName>
          <pointCloudCutoff>0.1</pointCloudCutoff>
          <pointCloudCutoffMax>10.0</pointCloudCutoffMax>
          <distortionK1>0.0</distortionK1>
          <distortionK2>0.0</distortionK2>
          <distortionK3>0.0</distortionK3>
          <distortionT1>0.0</distortionT1>
          <distortionT2>0.0</distortionT2>
        </plugin>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed">
      <parent>iris::base_link</parent>
      <child>camera_link</child>
    </joint>

    <!-- ========== 3D激光雷达 (Velodyne VLP-16风格) ========== -->
    <link name="lidar_link">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <iyy>0.0001</iyy>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <visual name="lidar_visual">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.07</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
        </material>
      </visual>
      <collision name="lidar_collision">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.07</length>
          </cylinder>
        </geometry>
      </collision>
      <sensor name="lidar_3d" type="ray">
        <pose>0 0 0.035 0 0 0</pose>
        <update_rate>10</update_rate>
        <visualize>true</visualize>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>16</samples>
              <resolution>1</resolution>
              <min_angle>-0.261799</min_angle>
              <max_angle>0.261799</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.3</min>
            <max>50.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="lidar_3d_plugin" filename="libgazebo_ros_velodyne_laser.so">
          <robotNamespace></robotNamespace>
          <topicName>velodyne_points</topicName>
          <frameName>lidar_link</frameName>
          <organize_cloud>false</organize_cloud>
          <min_range>0.3</min_range>
          <max_range>50.0</max_range>
          <gaussianNoise>0.01</gaussianNoise>
        </plugin>
      </sensor>
    </link>
    <joint name="lidar_joint" type="fixed">
      <parent>iris::base_link</parent>
      <child>lidar_link</child>
    </joint>

  </model>
</sdf>
EOF

echo ""
echo "=========================================="
echo "  模型创建完成!"
echo "=========================================="
echo ""
echo "传感器配置:"
echo "  - 深度相机: /camera/rgb/image_raw, /camera/depth/points"
echo "  - 3D激光雷达: /velodyne_points (16线, 360°)"
echo ""
echo "启动命令:"
echo "  roslaunch px4 mavros_posix_sitl.launch vehicle:=iris \\"
echo "      sdf:=\$(pwd)/Tools/sitl_gazebo/models/iris_full_sensor/iris_full_sensor.sdf \\"
echo "      world:=\$(pwd)/Tools/sitl_gazebo/worlds/hospital.world"
echo ""
