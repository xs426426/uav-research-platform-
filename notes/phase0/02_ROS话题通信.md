# 02. ROS 话题通信机制

## 一、核心概念

### 话题通信模型

```
ROS 的通信就像"广播电台"：

发布者 (Publisher)     →     话题 (Topic)     →     订阅者 (Subscriber)
   📻 电台                    📡 频道                   📻 收音机
```

### 形象比喻

```
想象一个微信群：

话题 (Topic) = 微信群
发布者 (Publisher) = 往群里发消息的人
订阅者 (Subscriber) = 在群里看消息的人

特点：
- 发布者不需要知道谁在看（解耦）
- 可以有多个发布者往同一个群发消息
- 可以有多个订阅者看同一个群的消息
- 消息是"广播"的，所有订阅者都能收到
```

## 二、代码详解

### 1. 创建订阅者

```python
# ============ 订阅者：监听飞控状态 ============
# 相当于：加入 "/mavros/state" 这个微信群，有新消息就调用 state_callback

self.state_sub = rospy.Subscriber(
    '/mavros/state',          # 话题名（群名）
    State,                     # 消息类型（消息格式）
    self.state_callback        # 回调函数（收到消息后做什么）
)

def state_callback(self, msg):
    """
    回调函数：每次话题有新消息时自动调用

    参数:
        msg: 收到的消息，类型是 State
    """
    self.current_state = msg

    # State 消息包含的字段：
    # msg.connected  - bool, 是否连接到飞控
    # msg.armed      - bool, 是否解锁
    # msg.mode       - string, 当前飞行模式 ("OFFBOARD", "MANUAL" 等)
```

### 2. 创建发布者

```python
# ============ 发布者：发送位置指令 ============
# 相当于：创建一个可以往群里发消息的账号

self.setpoint_pub = rospy.Publisher(
    '/mavros/setpoint_position/local',  # 话题名（群名）
    PoseStamped,                         # 消息类型
    queue_size=10                        # 消息队列大小（缓冲区）
)

def send_setpoint(self, x, y, z):
    """发送位置设定点"""

    # 1. 构造消息
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()  # 时间戳
    pose.header.frame_id = "map"           # 坐标系
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0          # 四元数，表示朝向

    # 2. 发布消息（往群里发）
    self.setpoint_pub.publish(pose)
```

## 三、无人机系统中的关键话题

### 话题列表

```
┌────────────────────────────────────┬──────────────┬─────────────────────┐
│             话题名                  │   消息类型   │        用途          │
├────────────────────────────────────┼──────────────┼─────────────────────┤
│ /mavros/state                      │ State        │ 飞控状态（连接/解锁）│
│ /mavros/local_position/pose        │ PoseStamped  │ 当前位置和姿态      │
│ /mavros/local_position/velocity    │ TwistStamped │ 当前速度            │
│ /mavros/imu/data                   │ Imu          │ IMU 数据            │
│ /mavros/setpoint_position/local    │ PoseStamped  │ 位置设定点（控制）  │
│ /mavros/setpoint_velocity/cmd_vel  │ TwistStamped │ 速度设定点（控制）  │
└────────────────────────────────────┴──────────────┴─────────────────────┘
```

### 数据流图

```
你的代码                    MAVROS                      PX4
   │                          │                          │
   │  /mavros/setpoint_xxx    │     MAVLink 消息         │
   ├─────────────────────────→├─────────────────────────→│ 控制指令
   │       (发布)             │      (转换)              │
   │                          │                          │
   │  /mavros/state           │     MAVLink 消息         │
   │←─────────────────────────┤←─────────────────────────┤ 状态反馈
   │       (订阅)             │      (转换)              │
   │                          │                          │
   │  /mavros/local_position  │     MAVLink 消息         │
   │←─────────────────────────┤←─────────────────────────┤ 位置反馈
   │       (订阅)             │      (转换)              │
```

## 四、消息类型详解

### 1. State 消息

```python
from mavros_msgs.msg import State

# 字段说明：
state.connected  # bool: 是否连接到飞控
state.armed      # bool: 是否解锁（电机是否可以转）
state.guided     # bool: 是否处于引导模式
state.mode       # string: 当前飞行模式
                 #   - "MANUAL": 手动模式
                 #   - "STABILIZED": 姿态稳定
                 #   - "OFFBOARD": 外部控制 ★
                 #   - "AUTO.LAND": 自动降落
                 #   - "AUTO.MISSION": 自动任务
```

### 2. PoseStamped 消息

```python
from geometry_msgs.msg import PoseStamped

# 字段说明：
pose.header.stamp      # 时间戳
pose.header.frame_id   # 坐标系名称

pose.pose.position.x   # X 坐标 (米)
pose.pose.position.y   # Y 坐标 (米)
pose.pose.position.z   # Z 坐标 (米)

pose.pose.orientation.x  # 四元数 x
pose.pose.orientation.y  # 四元数 y
pose.pose.orientation.z  # 四元数 z
pose.pose.orientation.w  # 四元数 w
                         # (1,0,0,0) 或 (0,0,0,1) 表示无旋转
```

## 五、命令行工具

```bash
# 查看所有话题
rostopic list

# 查看话题内容（实时）
rostopic echo /mavros/state

# 查看话题类型
rostopic type /mavros/state
# 输出: mavros_msgs/State

# 查看话题发布频率
rostopic hz /mavros/local_position/pose
# 输出: average rate: 30.0 (大约 30Hz)

# 查看话题详细信息
rostopic info /mavros/state
# 输出: 发布者、订阅者列表

# 查看消息类型的字段
rosmsg show mavros_msgs/State

# 手动发布一条消息（测试用）
rostopic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped "..."
```

## 六、回调函数机制

```python
# 回调函数是"被动调用"的

# 1. 当你创建订阅者时，ROS 会在后台创建一个线程
# 2. 这个线程持续监听话题
# 3. 一旦有新消息，就调用你的回调函数

# 示意图：
#
#   主线程                        ROS 后台线程
#     │                              │
#     │  创建订阅者                  │
#     ├─────────────────────────────→│
#     │                              │  开始监听话题
#     │                              │
#     │  继续执行其他代码            │  收到消息！
#     │                              │
#     │  ←──── 调用 callback() ──────┤
#     │                              │
#     │  继续执行其他代码            │  收到消息！
#     │                              │
#     │  ←──── 调用 callback() ──────┤

# 注意事项：
# - 回调函数应该快速返回，避免阻塞
# - 如果回调函数太慢，可能会丢失消息
# - 可以用 rospy.spin() 让主线程等待回调
```

## 七、总结

```
发布 (Publish)：
    publisher = rospy.Publisher('话题名', 消息类型, queue_size=10)
    publisher.publish(消息)

订阅 (Subscribe)：
    subscriber = rospy.Subscriber('话题名', 消息类型, 回调函数)

    def 回调函数(msg):
        # 处理收到的消息
        pass

核心理解：
    - 话题是"频道"，发布者和订阅者通过话题通信
    - 发布者不关心谁在订阅
    - 订阅者不关心谁在发布
    - 这种解耦让系统更灵活
```
