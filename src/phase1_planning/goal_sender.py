#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机控制面板 - 交互式控制工具

功能：
- 起飞/降落控制
- 手动输入目标坐标
- 预设目标点快速选择
- 航线飞行
- 状态监控

使用方法：
    python3 goal_sender.py
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
import sys
import threading
import time


class DroneController:
    """无人机控制面板"""

    def __init__(self):
        # 初始化 ROS 节点
        # anonymous=True 允许同时运行多个实例（节点名会加随机后缀）
        rospy.init_node('drone_controller', anonymous=True)

        # ============ 状态变量 ============
        self.current_state = State()  # 飞控状态（连接、解锁、模式）
        self.current_pose = None      # 当前位置

        # ============ 订阅者 ============
        # 订阅飞控状态话题
        self.state_sub = rospy.Subscriber(
            '/mavros/state',      # 话题名
            State,                # 消息类型
            self.state_callback   # 回调函数
        )

        # 订阅当前位置话题
        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            self.pose_callback
        )

        # ============ 发布者 ============
        # 发布目标点（给 path_planner_node）
        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal',
            PoseStamped,
            queue_size=10
        )

        # 发布位置设定点（直接给飞控，用于起飞）
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )

        # ============ 服务客户端 ============
        # 等待服务可用
        rospy.loginfo("等待 MAVROS 服务...")

        rospy.wait_for_service('/mavros/cmd/arming')
        # ServiceProxy 创建服务客户端，可以调用这个服务
        self.arming_client = rospy.ServiceProxy(
            '/mavros/cmd/arming',  # 服务名
            CommandBool            # 服务类型
        )

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy(
            '/mavros/set_mode',
            SetMode
        )

        # 等待发布者建立连接
        rospy.sleep(1.0)

        # ============ 预设目标点 ============
        self.presets = {
            '1': {'name': '前方悬停', 'x': 3.0, 'y': 0.0, 'z': 2.0},
            '2': {'name': '左前方', 'x': 3.0, 'y': 3.0, 'z': 2.0},
            '3': {'name': '右前方', 'x': 3.0, 'y': -3.0, 'z': 2.0},
            '4': {'name': '高空悬停', 'x': 0.0, 'y': 0.0, 'z': 5.0},
            '5': {'name': '绕圈起点', 'x': 5.0, 'y': 0.0, 'z': 3.0},
            '6': {'name': '远点测试', 'x': 10.0, 'y': 10.0, 'z': 3.0},
            '7': {'name': '返回原点', 'x': 0.0, 'y': 0.0, 'z': 2.0},
        }

        # 控制循环频率
        self.rate = rospy.Rate(20)  # 20Hz

        rospy.loginfo("无人机控制面板初始化完成")

    # ============ 回调函数 ============

    def state_callback(self, msg):
        """
        飞控状态回调

        State 消息包含：
        - connected: 是否连接到飞控
        - armed: 是否解锁（电机是否可以转）
        - mode: 当前飞行模式（MANUAL/OFFBOARD/AUTO.LAND 等）
        """
        self.current_state = msg

    def pose_callback(self, msg):
        """
        位置回调

        PoseStamped 消息包含：
        - header: 时间戳和坐标系
        - pose.position: x, y, z 坐标
        - pose.orientation: 四元数姿态
        """
        self.current_pose = msg

    # ============ 控制函数 ============

    def arm(self):
        """
        解锁无人机

        解锁后电机可以转动
        """
        req = CommandBoolRequest()
        req.value = True  # True=解锁, False=上锁

        try:
            resp = self.arming_client(req)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"解锁服务调用失败: {e}")
            return False

    def disarm(self):
        """上锁无人机"""
        req = CommandBoolRequest()
        req.value = False

        try:
            resp = self.arming_client(req)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"上锁服务调用失败: {e}")
            return False

    def set_mode(self, mode):
        """
        设置飞行模式

        常用模式：
        - OFFBOARD: 外部控制模式（你的代码控制）
        - AUTO.LAND: 自动降落
        - AUTO.RTL: 自动返航
        - MANUAL: 手动模式
        """
        req = SetModeRequest()
        req.custom_mode = mode

        try:
            resp = self.set_mode_client(req)
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr(f"模式切换服务调用失败: {e}")
            return False

    def send_setpoint(self, x, y, z):
        """
        发送位置设定点（直接给飞控）

        这是底层控制，直接发送给 PX4
        """
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.setpoint_pub.publish(pose)

    def send_goal(self, x, y, z):
        """
        发送目标点（给 path_planner_node）

        path_planner_node 会进行路径规划后执行
        """
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        print(f"\n>>> 已发送目标: x={x:.1f}, y={y:.1f}, z={z:.1f}")

    # ============ 高级控制 ============

    def takeoff(self, height=2.0):
        """
        起飞到指定高度

        流程：
        1. 预发送设定点（PX4 要求切换 OFFBOARD 前必须有设定点流）
        2. 切换到 OFFBOARD 模式
        3. 解锁
        4. 等待到达目标高度
        """
        print(f"\n>>> 开始起飞到 {height}m...")

        # 检查连接
        if not self.current_state.connected:
            print("!!! 飞控未连接")
            return False

        # 步骤 1: 预发送设定点
        # PX4 安全机制：切换 OFFBOARD 前必须已经在接收设定点（频率 > 2Hz）
        print("    预发送设定点...")
        for _ in range(100):  # 20Hz * 5秒 = 100次
            self.send_setpoint(0, 0, height)
            self.rate.sleep()

        # 步骤 2 & 3: 切换模式并解锁
        last_request = rospy.Time.now()

        while not rospy.is_shutdown():
            # 每 5 秒尝试一次
            if rospy.Time.now() - last_request > rospy.Duration(5.0):
                # 先切换模式
                if self.current_state.mode != "OFFBOARD":
                    if self.set_mode("OFFBOARD"):
                        print("    切换到 OFFBOARD 模式")
                    last_request = rospy.Time.now()
                # 再解锁
                elif not self.current_state.armed:
                    if self.arm():
                        print("    无人机已解锁")
                    last_request = rospy.Time.now()

            # 持续发送设定点（必须！否则会退出 OFFBOARD）
            self.send_setpoint(0, 0, height)

            # 检查是否到达高度
            if self.current_pose:
                current_z = self.current_pose.pose.position.z
                if current_z > height - 0.3:
                    print(f"    到达目标高度: {current_z:.1f}m")
                    print(">>> 起飞完成!")
                    return True

            self.rate.sleep()

        return False

    def land(self):
        """
        降落

        切换到 AUTO.LAND 模式，PX4 会自动控制降落
        """
        print("\n>>> 开始降落...")

        if self.set_mode("AUTO.LAND"):
            print("    切换到 AUTO.LAND 模式")
            print(">>> 降落中...")
            return True
        else:
            print("!!! 降落失败")
            return False

    def hover(self):
        """
        原地悬停

        在当前位置悬停
        """
        if self.current_pose is None:
            print("!!! 位置未知，无法悬停")
            return

        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z

        print(f"\n>>> 原地悬停在 ({x:.1f}, {y:.1f}, {z:.1f})")
        self.send_setpoint(x, y, z)

    # ============ 菜单和交互 ============

    def print_status(self):
        """打印当前状态"""
        print("\n" + "-"*50)
        print("【当前状态】")
        print(f"  连接: {'✓ 已连接' if self.current_state.connected else '✗ 未连接'}")
        print(f"  解锁: {'✓ 已解锁' if self.current_state.armed else '✗ 未解锁'}")
        print(f"  模式: {self.current_state.mode}")

        if self.current_pose:
            p = self.current_pose.pose.position
            print(f"  位置: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
        else:
            print("  位置: 未知")
        print("-"*50)

    def print_menu(self):
        """打印菜单"""
        print("\n" + "="*50)
        print("        无人机控制面板")
        print("="*50)

        print("\n【飞行控制】")
        print("  t. 起飞 (Takeoff)")
        print("  l. 降落 (Land)")
        print("  h. 悬停 (Hover)")

        print("\n【预设目标点】")
        for key, preset in self.presets.items():
            print(f"  {key}. {preset['name']:10} -> ({preset['x']}, {preset['y']}, {preset['z']})")

        print("\n【其他操作】")
        print("  c. 自定义坐标输入")
        print("  w. 航线飞行")
        print("  i. 查看状态 (Info)")
        print("  q. 退出")
        print("-"*50)

    def get_custom_goal(self):
        """获取自定义目标坐标"""
        try:
            print("\n输入目标坐标（单位：米）")
            x = float(input("  X (前方): "))
            y = float(input("  Y (左方): "))
            z = float(input("  Z (高度): "))
            return x, y, z
        except ValueError:
            print("!!! 输入无效，请输入数字")
            return None

    def send_waypoints(self):
        """发送多个航点（航线飞行）"""
        print("\n【航线模式】")
        print("预设航线：")
        print("  1. 正方形航线")
        print("  2. 三角形航线")
        print("  3. 自定义航点")

        choice = input("选择航线: ").strip()

        if choice == '1':
            waypoints = [
                (3, 0, 2),
                (3, 3, 2),
                (0, 3, 2),
                (0, 0, 2),
            ]
        elif choice == '2':
            waypoints = [
                (4, 0, 2),
                (2, 3, 2),
                (0, 0, 2),
            ]
        elif choice == '3':
            waypoints = []
            print("输入航点坐标，输入 'done' 结束：")
            while True:
                try:
                    inp = input(f"  航点 {len(waypoints)+1} (x y z): ")
                    if inp.lower() == 'done':
                        break
                    parts = inp.split()
                    if len(parts) == 3:
                        waypoints.append((float(parts[0]), float(parts[1]), float(parts[2])))
                except ValueError:
                    print("  格式错误，请输入: x y z")
        else:
            print("无效选择")
            return

        if not waypoints:
            print("没有航点")
            return

        print(f"\n将依次发送 {len(waypoints)} 个航点")
        print("提示：按 Enter 发送下一个航点")

        for i, (x, y, z) in enumerate(waypoints):
            input(f"\n按 Enter 发送航点 {i+1}/{len(waypoints)}: ({x}, {y}, {z})")
            self.send_goal(x, y, z)

    def run(self):
        """主循环"""
        print("\n" + "="*50)
        print("  无人机控制面板已启动")
        print("="*50)

        # 等待飞控连接
        print("\n等待飞控连接...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()
        print("飞控已连接!")

        self.print_status()

        while not rospy.is_shutdown():
            self.print_menu()
            choice = input("请选择: ").strip().lower()

            if choice == 'q':
                print("\n退出程序")
                break
            elif choice == 't':
                height = input("起飞高度 (默认 2.0m): ").strip()
                height = float(height) if height else 2.0
                self.takeoff(height)
            elif choice == 'l':
                self.land()
            elif choice == 'h':
                self.hover()
            elif choice == 'c':
                result = self.get_custom_goal()
                if result:
                    self.send_goal(*result)
            elif choice == 'w':
                self.send_waypoints()
            elif choice == 'i':
                self.print_status()
            elif choice in self.presets:
                preset = self.presets[choice]
                print(f"\n选择: {preset['name']}")
                self.send_goal(preset['x'], preset['y'], preset['z'])
            else:
                print("\n!!! 无效选择，请重试")


def main():
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n\n用户中断")


if __name__ == '__main__':
    main()
