# Phase 0: 系统地基

> 目标：搭建稳定的开发环境，实现无人机基础控制

## 🎯 本阶段验收标准

1. Gazebo 中无人机能 **起飞 → 悬停 → 降落**
2. 能用 ROS topic 控制无人机位置
3. 理解 PX4 + MAVROS 通信架构

## 📚 理论知识

### 1. 系统架构总览

```
┌─────────────────────────────────────────────────────────────┐
│                      你的控制节点                            │
│                    (Python/C++ ROS Node)                    │
└─────────────────────┬───────────────────────────────────────┘
                      │ ROS Topics/Services
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                       MAVROS                                 │
│              (ROS ↔ MAVLink 桥接)                           │
└─────────────────────┬───────────────────────────────────────┘
                      │ MAVLink 协议
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                     PX4 SITL                                 │
│                  (飞控固件仿真)                              │
└─────────────────────┬───────────────────────────────────────┘
                      │ 传感器/执行器接口
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                      Gazebo                                  │
│                   (物理仿真引擎)                             │
└─────────────────────────────────────────────────────────────┘
```

### 2. 关键概念

#### PX4 飞行模式
| 模式 | 说明 | 使用场景 |
|------|------|----------|
| MANUAL | 手动控制 | 遥控器直接控制 |
| STABILIZED | 姿态稳定 | 半自动飞行 |
| ALTITUDE | 高度保持 | 定高飞行 |
| POSITION | 位置保持 | 定点悬停 |
| OFFBOARD | 外部控制 | **我们主要用这个** |
| AUTO.MISSION | 自主任务 | 航点飞行 |

#### MAVROS 重要话题
```bash
# 状态信息
/mavros/state              # 飞控状态（连接、解锁、模式）
/mavros/local_position/pose # 本地位置
/mavros/imu/data           # IMU数据

# 控制指令
/mavros/setpoint_position/local  # 位置设定点
/mavros/setpoint_velocity/cmd_vel # 速度设定点
/mavros/setpoint_raw/attitude    # 姿态设定点
```

### 3. OFFBOARD 模式工作原理

```
1. 持续发送设定点 (>2Hz，推荐20Hz)
       ↓
2. 请求切换到 OFFBOARD 模式
       ↓
3. 请求解锁 (ARM)
       ↓
4. 飞控执行你的指令
```

⚠️ **重要**：必须在切换 OFFBOARD 之前就开始发送设定点！

## 🔧 环境安装

### Step 1: ROS Noetic 安装

运行脚本：`scripts/install_ros.sh`

### Step 2: PX4 + Gazebo 安装

运行脚本：`scripts/install_px4.sh`

### Step 3: MAVROS 安装

运行脚本：`scripts/install_mavros.sh`

## 💻 代码实践

### 实践1：启动仿真环境

```bash
# 终端1：启动 PX4 SITL + Gazebo
cd ~/PX4-Autopilot
make px4_sitl gazebo

# 终端2：启动 MAVROS
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

# 终端3：查看话题
rostopic list
rostopic echo /mavros/state
```

### 实践2：手动控制测试

```bash
# 在 QGroundControl 或命令行中
# 切换到 POSCTL 模式，手动测试起飞
```

### 实践3：OFFBOARD 控制（代码见 src/phase0_foundation/）

Python 版本：`offboard_control.py`
C++ 版本：`offboard_control_node.cpp`

## 📝 学习检查清单

- [ ] 理解 ROS 节点、话题、服务的概念
- [ ] 理解 PX4 的飞行模式
- [ ] 理解 MAVROS 的作用
- [ ] 能解释 OFFBOARD 模式的工作流程
- [ ] 成功运行起飞-悬停-降落 demo
- [ ] 能修改代码让无人机飞到指定位置

## 🐛 常见问题

### Q1: OFFBOARD 模式切换失败
**原因**：设定点发送频率不够或未提前发送
**解决**：确保在切换模式前已经以 >2Hz 的频率发送设定点至少 2 秒

### Q2: 无人机无法解锁
**原因**：安全检查未通过
**解决**：检查 GPS 状态、遥控器连接、电池电量

### Q3: Gazebo 启动很慢/卡住
**原因**：首次启动需要下载模型
**解决**：等待下载完成，或手动下载模型到 ~/.gazebo/models/

## 📖 延伸阅读

1. [PX4 开发者指南](https://docs.px4.io/main/en/development/development.html)
2. [MAVROS 官方文档](http://wiki.ros.org/mavros)
3. [MAVLink 协议](https://mavlink.io/en/)
