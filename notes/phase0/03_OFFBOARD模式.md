# 03. OFFBOARD 控制模式

## 一、PX4 飞行模式总览

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           PX4 飞行模式                                   │
├─────────────────┬───────────────────────────────────────────────────────┤
│  MANUAL         │  完全手动控制，遥控器直接控制电机                       │
├─────────────────┼───────────────────────────────────────────────────────┤
│  STABILIZED     │  姿态稳定模式，遥控器控制姿态角度                       │
├─────────────────┼───────────────────────────────────────────────────────┤
│  ALTITUDE       │  定高模式，自动保持高度，遥控器控制水平移动             │
├─────────────────┼───────────────────────────────────────────────────────┤
│  POSITION       │  定点模式，自动保持位置，遥控器控制目标位置             │
├─────────────────┼───────────────────────────────────────────────────────┤
│  ★ OFFBOARD     │  外部控制模式，【由你的代码控制无人机】                 │
├─────────────────┼───────────────────────────────────────────────────────┤
│  AUTO.TAKEOFF   │  自动起飞                                             │
├─────────────────┼───────────────────────────────────────────────────────┤
│  AUTO.LAND      │  自动降落                                             │
├─────────────────┼───────────────────────────────────────────────────────┤
│  AUTO.MISSION   │  自动执行预设航点任务                                  │
├─────────────────┼───────────────────────────────────────────────────────┤
│  AUTO.RTL       │  自动返航 (Return To Launch)                          │
└─────────────────┴───────────────────────────────────────────────────────┘
```

## 二、OFFBOARD 模式详解

### 什么是 OFFBOARD 模式

```
OFFBOARD = Off-Board Control = 外部控制

意思是：飞控把控制权交给外部计算机（你的代码）

对比：
┌────────────────────────────────────────────────────────────────┐
│  其他模式：飞控自己决定怎么飞                                    │
│  OFFBOARD：飞控听你的代码指挥                                    │
└────────────────────────────────────────────────────────────────┘
```

### OFFBOARD 模式可以控制什么

```
1. 位置控制 (Position)
   - 发送目标位置 (x, y, z)
   - 飞控自动规划如何到达
   - 话题: /mavros/setpoint_position/local

2. 速度控制 (Velocity)
   - 发送目标速度 (vx, vy, vz)
   - 飞控按指定速度飞行
   - 话题: /mavros/setpoint_velocity/cmd_vel

3. 姿态控制 (Attitude)
   - 发送目标姿态（横滚、俯仰、偏航）
   - 更底层的控制
   - 话题: /mavros/setpoint_attitude/attitude

4. 推力控制 (Thrust)
   - 直接控制电机推力
   - 最底层的控制
   - 需要自己实现姿态控制算法
```

## 三、OFFBOARD 模式状态机

```
                    ┌──────────────┐
                    │   未连接      │
                    └──────┬───────┘
                           │ MAVROS 连接成功
                           ▼
                    ┌──────────────┐
                    │   已连接      │
                    │   未解锁      │
                    │   非OFFBOARD  │
                    └──────┬───────┘
                           │ ① 持续发送设定点 (>2Hz, 持续>2秒)
                           │ ② 请求切换 OFFBOARD 模式
                           ▼
                    ┌──────────────┐
                    │   已连接      │
                    │   未解锁      │
                    │   OFFBOARD    │
                    └──────┬───────┘
                           │ ③ 请求解锁 (ARM)
                           ▼
                    ┌──────────────┐
                    │   已连接      │
                    │   ★ 已解锁   │  ← 无人机开始执行你的指令！
                    │   OFFBOARD    │
                    └──────────────┘
```

## 四、为什么要先发设定点再切模式

### PX4 的安全设计

```
PX4 的设计哲学：安全第一

场景分析：如果允许先切 OFFBOARD，再发设定点...

  时刻 0: 切换到 OFFBOARD
  时刻 1: 无人机：我该飞哪？（没有设定点）
  时刻 2: 无人机：不知道该干嘛，可能失控！危险！

正确做法：

  时刻 -2: 开始发送设定点 (目标: 当前位置)
  时刻 -1: 继续发送设定点
  时刻 0:  切换到 OFFBOARD
  时刻 1:  无人机：我该飞哪？
  时刻 2:  无人机：收到设定点了，去那个位置！✓
```

### 具体要求

```
OFFBOARD 模式的硬性要求：

┌────────────────────────────────────────────────────────────────┐
│ 1. 切换前：必须已经在接收设定点                                  │
│    - 频率 > 2Hz                                                 │
│    - 持续时间 > 2 秒                                            │
│    - 否则：切换失败                                             │
├────────────────────────────────────────────────────────────────┤
│ 2. 切换后：必须持续发送设定点                                    │
│    - 频率 > 2Hz（推荐 20Hz 以上）                               │
│    - 不能中断                                                   │
│    - 否则：自动退出 OFFBOARD                                    │
├────────────────────────────────────────────────────────────────┤
│ 3. 超时机制：设定点中断超过 500ms                               │
│    - 触发故障保护 (Failsafe)                                    │
│    - 可能自动降落或悬停                                         │
└────────────────────────────────────────────────────────────────┘
```

## 五、代码实现详解

### 完整的起飞流程

```python
def takeoff(self, height=2.0):
    """
    起飞到指定高度

    流程：
    1. 预发送设定点
    2. 切换 OFFBOARD 模式
    3. 解锁
    4. 等待到达目标高度
    """

    # ========== 步骤 1：预发送设定点 ==========
    # 为什么：PX4 要求切换 OFFBOARD 前必须已经在接收设定点
    # 频率：20Hz（每 50ms 发一次）
    # 时长：5 秒（100 次）

    rospy.loginfo("预发送设定点...")
    for _ in range(100):  # 20Hz × 5秒 = 100次
        self.send_setpoint(0, 0, height)
        self.rate.sleep()  # rate = 20Hz，所以 sleep 50ms

    # ========== 步骤 2 & 3：切换模式 + 解锁 ==========

    last_request = rospy.Time.now()

    while not rospy.is_shutdown():

        # 每 5 秒尝试一次（避免频繁请求）
        if rospy.Time.now() - last_request > rospy.Duration(5.0):

            # 检查当前模式
            if self.current_state.mode != "OFFBOARD":
                # 还不是 OFFBOARD，请求切换
                if self.set_mode("OFFBOARD"):
                    rospy.loginfo("切换到 OFFBOARD 模式")
                last_request = rospy.Time.now()

            # 已经是 OFFBOARD，检查是否解锁
            elif not self.current_state.armed:
                # 还没解锁，请求解锁
                if self.arm():
                    rospy.loginfo("无人机已解锁")
                last_request = rospy.Time.now()

        # ★★★ 关键：无论如何都要持续发送设定点 ★★★
        self.send_setpoint(0, 0, height)

        # 检查是否到达目标高度
        if abs(self.current_pose.pose.position.z - height) < 0.1:
            rospy.loginfo(f"到达目标高度: {height}m")
            return True

        self.rate.sleep()  # 保持 20Hz
```

### 关键服务调用

```python
# ========== 切换飞行模式 ==========
from mavros_msgs.srv import SetMode, SetModeRequest

def set_mode(self, mode):
    """
    切换飞行模式

    参数:
        mode: 模式字符串
              - "OFFBOARD": 外部控制
              - "AUTO.LAND": 自动降落
              - "AUTO.RTL": 自动返航
    """
    req = SetModeRequest()
    req.custom_mode = mode

    try:
        resp = self.set_mode_client(req)
        return resp.mode_sent
    except rospy.ServiceException as e:
        rospy.logerr(f"模式切换失败: {e}")
        return False


# ========== 解锁/上锁 ==========
from mavros_msgs.srv import CommandBool, CommandBoolRequest

def arm(self):
    """解锁无人机（允许电机转动）"""
    req = CommandBoolRequest()
    req.value = True  # True=解锁, False=上锁

    try:
        resp = self.arming_client(req)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"解锁失败: {e}")
        return False
```

## 六、常见问题

### 问题 1：OFFBOARD 模式切换失败

```
原因：
  - 设定点发送频率不够 (< 2Hz)
  - 设定点发送时间不够 (< 2秒)
  - 没有提前发送设定点

解决：
  - 确保 rate >= 20Hz
  - 预发送设定点至少 2 秒
  - 检查话题是否正确
```

### 问题 2：无人机无法解锁

```
原因：
  - 安全检查未通过
  - 没有遥控器连接
  - 参数限制

解决：
  # 在 PX4 终端设置参数
  pxh> param set COM_RCL_EXCEPT 4   # 允许无遥控器使用 OFFBOARD
  pxh> param set COM_ARM_WO_GPS 1   # 允许无 GPS 解锁（仿真环境）
```

### 问题 3：OFFBOARD 模式自动退出

```
原因：
  - 设定点发送中断超过 500ms
  - 代码执行太慢

解决：
  - 确保主循环持续运行
  - 避免在循环中做耗时操作
  - 可以把耗时操作放到单独线程
```

## 七、总结

```
OFFBOARD 模式核心要点：

1. 切换前：先发设定点 (>2Hz, >2秒)
2. 切换后：持续发设定点 (>2Hz)
3. 中断 >500ms：自动退出

代码模板：

    # 1. 预发送
    for i in range(100):
        send_setpoint(x, y, z)
        rate.sleep()

    # 2. 主循环
    while not done:
        # 尝试切换模式和解锁
        if need_switch_mode:
            set_mode("OFFBOARD")
        if need_arm:
            arm()

        # ★ 关键：无论如何都发设定点
        send_setpoint(x, y, z)

        rate.sleep()
```
