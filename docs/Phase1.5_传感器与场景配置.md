# Phase 1.5: 传感器与测试场景配置

> 为无人机添加深度相机/激光雷达，并搭建测试环境

---

## 一、测试场景启动

### 方法 1：使用现成的 warehouse 场景

```bash
# 终端 1：启动带仓库场景的 PX4 仿真
cd ~/PX4-Autopilot
make px4_sitl gazebo_iris__warehouse
```

如果 warehouse 场景不可用，尝试：

```bash
# 或者使用 baylands（室外场景）
make px4_sitl gazebo_iris__baylands

# 或者 ksql_airport（机场）
make px4_sitl gazebo_iris__ksql_airport
```

### 方法 2：创建自定义障碍物场景

在 Ubuntu 中创建自定义世界文件：

```bash
# 创建自定义世界文件
cat > ~/PX4-Autopilot/Tools/sitl_gazebo/worlds/obstacle_test.world << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <!-- 基础设置 -->
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- 物理引擎 -->
    <physics name='default_physics' default='0' type='ode'>
      <gravity>0 0 -9.8066</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
      </ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

    <!-- ========== 障碍物 ========== -->

    <!-- 障碍物 1: 立方体墙 (5,5) 位置 -->
    <model name="obstacle_wall_1">
      <static>true</static>
      <pose>5 5 2.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>3 3 5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>3 3 5</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 障碍物 2: 高柱子 (10, 0) 位置 -->
    <model name="obstacle_pillar">
      <static>true</static>
      <pose>10 0 4 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>1.5</radius>
              <length>8</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>1.5</radius>
              <length>8</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 障碍物 3: 长墙 (3, -5) 位置 -->
    <model name="obstacle_long_wall">
      <static>true</static>
      <pose>3 -5 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>8 1 3</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>8 1 3</size></box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
            <diffuse>0.2 0.2 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 障碍物 4: 低矮障碍 (7, 7) -->
    <model name="obstacle_low">
      <static>true</static>
      <pose>7 7 0.75 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>2 4 1.5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>2 4 1.5</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.2 1</ambient>
            <diffuse>0.8 0.8 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 障碍物 5: 门形通道 -->
    <model name="gate_left">
      <static>true</static>
      <pose>15 3 2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 4</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 4</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <model name="gate_right">
      <static>true</static>
      <pose>15 -3 2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 4</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 4</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <model name="gate_top">
      <static>true</static>
      <pose>15 0 4.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 7 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 7 1</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
EOF
```

启动自定义场景：

```bash
# 需要先设置环境变量
export PX4_SITL_WORLD=obstacle_test
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

---

## 二、为无人机添加传感器

### 方法：使用带传感器的无人机模型

PX4 提供了预配置的带传感器无人机模型：

```bash
# 带深度相机的无人机（推荐！）
cd ~/PX4-Autopilot
make px4_sitl gazebo_iris_depth_camera

# 或者带激光雷达的无人机
make px4_sitl gazebo_iris_rplidar

# 组合：深度相机 + 自定义场景
export PX4_SITL_WORLD=obstacle_test
make px4_sitl gazebo_iris_depth_camera
```

### 查看可用的传感器配置

```bash
# 列出所有可用的无人机模型
ls ~/PX4-Autopilot/Tools/sitl_gazebo/models/ | grep iris
```

常见配置：
- `iris` - 基础四旋翼
- `iris_depth_camera` - 带深度相机
- `iris_rplidar` - 带 2D 激光雷达
- `iris_opt_flow` - 带光流传感器

---

## 三、传感器数据话题

### 深度相机话题

```bash
# 查看深度相机相关话题
rostopic list | grep -E "(depth|camera|image)"

# 常见话题：
# /camera/depth/image_raw      - 深度图像（原始）
# /camera/depth/points         - 点云数据
# /camera/rgb/image_raw        - RGB 图像
# /camera/depth/camera_info    - 相机参数
```

### 激光雷达话题

```bash
# 查看激光雷达话题
rostopic list | grep -E "(scan|laser|lidar)"

# 常见话题：
# /laser/scan                  - 2D 激光扫描
# /velodyne_points            - 3D 点云（如果有）
```

### 查看数据

```bash
# 实时查看点云数据
rostopic echo /camera/depth/points

# 查看数据发布频率
rostopic hz /camera/depth/image_raw

# 使用 RViz 可视化
rviz
# 在 RViz 中添加:
# - PointCloud2: /camera/depth/points
# - Image: /camera/depth/image_raw
```

---

## 四、完整启动流程（带传感器+障碍物场景）

### 准备工作（只需执行一次）

```bash
# 1. 创建自定义世界文件（上面的 obstacle_test.world）

# 2. 确保 catkin 工作空间已编译
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 每次启动流程

```bash
# === 终端 1：PX4 + Gazebo（带深度相机和障碍物场景）===
cd ~/PX4-Autopilot
export PX4_SITL_WORLD=obstacle_test
make px4_sitl gazebo_iris_depth_camera

# 等待出现 pxh> 后设置参数
# pxh> param set COM_RCL_EXCEPT 4

# === 终端 2：MAVROS ===
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

# === 终端 3：RViz 可视化（可选）===
rviz

# === 终端 4：控制脚本 ===
cd ~/uav-research-platform-/src/phase1_planning
python3 goal_sender.py
```

---

## 五、RViz 配置

在 RViz 中添加以下显示项：

1. **Fixed Frame**: 设置为 `map`

2. **TF**: 显示坐标系关系

3. **PointCloud2**:
   - Topic: `/camera/depth/points`
   - Color: 按深度着色

4. **Image**:
   - Topic: `/camera/depth/image_raw`

5. **Path**:
   - Topic: `/planned_path`
   - 显示规划的路径

6. **MarkerArray**:
   - Topic: `/planning_markers`
   - 显示障碍物和航点

保存配置：File → Save Config As → `~/uav_viz.rviz`

以后可以直接加载：
```bash
rviz -d ~/uav_viz.rviz
```

---

## 六、场景布局图

```
俯视图（XY 平面，Z 向上）

     Y
     ↑
     │
  10 │            ○ 高柱子(10,0)
     │           r=1.5
   7 │                    ■ 低障碍(7,7)
     │
   5 │    ■ 红墙(5,5)
     │    3x3x5
   3 │                          ┃ 门(15,3)
     │                          ┃
   0 ●────────────────○─────────╋────────→ X
     │ 起点(0,0)               ┃
  -3 │                          ┃ 门(15,-3)
     │
  -5 │  ████████ 蓝墙(3,-5)
     │   8x1x3
     │
     0    5    10   15   20

颜色说明：
■ 红墙 - 大立方体
○ 绿柱 - 圆柱体
█ 蓝墙 - 长条形
┃ 灰门 - 门形通道
```

---

## 七、验证传感器工作

```bash
# 1. 检查话题是否存在
rostopic list | grep camera

# 2. 检查数据是否在发布
rostopic hz /camera/depth/image_raw
# 应该显示 ~30Hz

# 3. 查看一帧数据
rostopic echo /camera/depth/image_raw -n 1

# 4. 如果使用点云
rostopic echo /camera/depth/points -n 1
```

---

## 下一步

传感器和场景配置完成后，我们可以：

1. **Phase 2**: 使用激光雷达做里程计（FAST-LIO2）
2. **Phase 3**: 使用深度相机做视觉 SLAM（ORB-SLAM3）
3. **Phase 4**: 构建环境地图（Octomap）

先验证传感器数据正常，然后继续下一阶段！
