#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OFFBOARD 控制示例 - 起飞、悬停、降落
====================================

这是 Phase 0 的核心代码，演示如何使用 MAVROS 控制无人机。

关键概念：
1. OFFBOARD 模式要求持续发送设定点 (>2Hz)
2. 必须先发送设定点，再切换模式
3. 切换到 OFFBOARD 后才能解锁

运行方式：
1. 启动 PX4 SITL: cd ~/PX4-Autopilot && make px4_sitl gazebo
2. 启动 MAVROS: roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
3. 运行本脚本: rosrun uav_control offboard_control.py

作者: UAV Research Platform
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest


class OffboardControl:
    """OFFBOARD 控制类"""

    def __init__(self):
        # 初始化节点
        rospy.init_node('offboard_control_node', anonymous=True)

        # 当前状态
        self.current_state = State()
        self.current_pose = PoseStamped()

        # ============ 订阅者 ============
        # 订阅飞控状态
        self.state_sub = rospy.Subscriber(
            '/mavros/state',
            State,
            self.state_callback
        )

        # 订阅当前位置
        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            self.pose_callback
        )

        # ============ 发布者 ============
        # 发布位置设定点
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )

        # ============ 服务客户端 ============
        # 解锁服务
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy(
            '/mavros/cmd/arming',
            CommandBool
        )

        # 模式切换服务
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy(
            '/mavros/set_mode',
            SetMode
        )

        # 控制频率 (20Hz)
        self.rate = rospy.Rate(20)

        rospy.loginfo("OFFBOARD 控制节点初始化完成")

    def state_callback(self, msg):
        """状态回调函数"""
        self.current_state = msg

    def pose_callback(self, msg):
        """位置回调函数"""
        self.current_pose = msg

    def wait_for_connection(self):
        """等待与飞控建立连接"""
        rospy.loginfo("等待与飞控建立连接...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()
        rospy.loginfo("飞控连接成功!")

    def send_setpoint(self, x, y, z):
        """
        发送位置设定点

        参数:
            x, y, z: 目标位置 (NED坐标系，单位：米)
        """
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # 保持水平姿态 (四元数)
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        self.setpoint_pub.publish(pose)

    def set_mode(self, mode):
        """
        切换飞行模式

        参数:
            mode: 模式字符串，如 "OFFBOARD", "AUTO.LAND"
        """
        mode_req = SetModeRequest()
        mode_req.custom_mode = mode

        try:
            response = self.set_mode_client(mode_req)
            if response.mode_sent:
                rospy.loginfo(f"模式切换成功: {mode}")
                return True
        except rospy.ServiceException as e:
            rospy.logerr(f"模式切换失败: {e}")
        return False

    def arm(self):
        """解锁无人机"""
        arm_req = CommandBoolRequest()
        arm_req.value = True

        try:
            response = self.arming_client(arm_req)
            if response.success:
                rospy.loginfo("无人机解锁成功!")
                return True
        except rospy.ServiceException as e:
            rospy.logerr(f"解锁失败: {e}")
        return False

    def disarm(self):
        """上锁无人机"""
        arm_req = CommandBoolRequest()
        arm_req.value = False

        try:
            response = self.arming_client(arm_req)
            if response.success:
                rospy.loginfo("无人机上锁成功!")
                return True
        except rospy.ServiceException as e:
            rospy.logerr(f"上锁失败: {e}")
        return False

    def takeoff(self, height=2.0):
        """
        起飞到指定高度

        参数:
            height: 目标高度 (米)
        """
        rospy.loginfo(f"准备起飞到 {height} 米...")

        # ============ 关键步骤 1: 预发送设定点 ============
        # 必须在切换 OFFBOARD 之前发送设定点
        rospy.loginfo("预发送设定点...")
        for _ in range(100):  # 发送约 5 秒
            if rospy.is_shutdown():
                return False
            self.send_setpoint(0, 0, height)
            self.rate.sleep()

        # ============ 关键步骤 2: 切换到 OFFBOARD 模式 ============
        last_request = rospy.Time.now()

        while not rospy.is_shutdown():
            # 每 5 秒尝试一次切换/解锁
            if rospy.Time.now() - last_request > rospy.Duration(5.0):
                # 尝试切换到 OFFBOARD
                if self.current_state.mode != "OFFBOARD":
                    if self.set_mode("OFFBOARD"):
                        last_request = rospy.Time.now()

                # 尝试解锁
                elif not self.current_state.armed:
                    if self.arm():
                        last_request = rospy.Time.now()

            # 持续发送设定点
            self.send_setpoint(0, 0, height)

            # 检查是否到达目标高度
            current_z = self.current_pose.pose.position.z
            if abs(current_z - height) < 0.1:  # 误差小于 10cm
                rospy.loginfo(f"已到达目标高度: {current_z:.2f} 米")
                return True

            self.rate.sleep()

        return False

    def go_to(self, x, y, z, tolerance=0.2):
        """
        飞到指定位置

        参数:
            x, y, z: 目标位置
            tolerance: 到达判定阈值 (米)
        """
        rospy.loginfo(f"前往位置: ({x}, {y}, {z})")

        while not rospy.is_shutdown():
            self.send_setpoint(x, y, z)

            # 计算距离
            dx = self.current_pose.pose.position.x - x
            dy = self.current_pose.pose.position.y - y
            dz = self.current_pose.pose.position.z - z
            distance = (dx**2 + dy**2 + dz**2) ** 0.5

            if distance < tolerance:
                rospy.loginfo(f"已到达位置: ({x}, {y}, {z})")
                return True

            self.rate.sleep()

        return False

    def hover(self, duration=5.0):
        """
        原地悬停

        参数:
            duration: 悬停时间 (秒)
        """
        rospy.loginfo(f"悬停 {duration} 秒...")

        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                rospy.loginfo("悬停完成")
                return True

            self.send_setpoint(x, y, z)
            self.rate.sleep()

        return False

    def land(self):
        """降落"""
        rospy.loginfo("开始降落...")

        # 方法1: 使用 AUTO.LAND 模式
        if self.set_mode("AUTO.LAND"):
            # 等待降落完成
            while not rospy.is_shutdown():
                if self.current_pose.pose.position.z < 0.1:
                    rospy.loginfo("降落完成!")
                    return True
                self.rate.sleep()

        return False

    def run_demo(self):
        """
        运行演示任务: 起飞 → 悬停 → 移动 → 降落
        """
        rospy.loginfo("=" * 50)
        rospy.loginfo("OFFBOARD 控制演示")
        rospy.loginfo("=" * 50)

        # 等待连接
        self.wait_for_connection()

        # 任务序列
        # 1. 起飞到 2 米
        if not self.takeoff(height=2.0):
            rospy.logerr("起飞失败!")
            return

        # 2. 悬停 5 秒
        self.hover(duration=5.0)

        # 3. 飞到指定位置
        self.go_to(2, 2, 2)  # 前进 2m, 右移 2m
        self.hover(duration=3.0)

        # 4. 返回原点上方
        self.go_to(0, 0, 2)
        self.hover(duration=3.0)

        # 5. 降落
        self.land()

        rospy.loginfo("演示完成!")


def main():
    try:
        controller = OffboardControl()
        controller.run_demo()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
