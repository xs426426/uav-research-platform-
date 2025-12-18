#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机控制面板 v2.0

改进：
1. 起飞后持续保持 OFFBOARD 模式
2. 支持随时发送新目标（中断当前任务）
3. 直接控制飞行，不依赖 path_planner_node
4. 可选择是否使用路径规划

使用方法：
    python3 goal_sender.py
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
import threading


class DroneController:
    """无人机控制面板 v2.0"""

    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('drone_controller', anonymous=True)

        # ============ 状态变量 ============
        self.current_state = State()
        self.current_pose = None
        self.target_pose = None           # 当前目标位置
        self.is_flying = False            # 是否在飞行中
        self.use_planner = False          # 是否使用路径规划器

        # ============ 订阅者 ============
        self.state_sub = rospy.Subscriber(
            '/mavros/state',
            State,
            self.state_callback
        )

        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            self.pose_callback
        )

        # ============ 发布者 ============
        # 发送目标给路径规划器（如果使用）
        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal',
            PoseStamped,
            queue_size=10
        )

        # 直接发送设定点给飞控
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )

        # ============ 服务客户端 ============
        rospy.loginfo("等待 MAVROS 服务...")
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.sleep(1.0)

        # ============ 预设目标点 ============
        self.presets = {
            '1': {'name': '前方 3m', 'x': 3.0, 'y': 0.0, 'z': 2.0},
            '2': {'name': '左前方', 'x': 3.0, 'y': 3.0, 'z': 2.0},
            '3': {'name': '右前方', 'x': 3.0, 'y': -3.0, 'z': 2.0},
            '4': {'name': '高空 5m', 'x': 0.0, 'y': 0.0, 'z': 5.0},
            '5': {'name': '远点 10m', 'x': 10.0, 'y': 0.0, 'z': 3.0},
            '6': {'name': '斜向远点', 'x': 10.0, 'y': 10.0, 'z': 3.0},
            '7': {'name': '返回原点', 'x': 0.0, 'y': 0.0, 'z': 2.0},
        }

        self.rate = rospy.Rate(20)

        # ============ 启动后台控制线程 ============
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        rospy.loginfo("无人机控制面板 v2.0 初始化完成")

    # ============ 回调函数 ============

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    # ============ 核心控制逻辑 ============

    def _control_loop(self):
        """
        后台控制循环（独立线程）

        持续运行，保证：
        1. 一直发送设定点（维持 OFFBOARD）
        2. 可以随时更新目标点
        """
        while not rospy.is_shutdown():
            if self.is_flying and self.target_pose is not None:
                # 发送当前目标位置
                self._send_setpoint(
                    self.target_pose[0],
                    self.target_pose[1],
                    self.target_pose[2]
                )
            elif self.is_flying and self.current_pose is not None:
                # 没有目标时，保持当前位置（悬停）
                self._send_setpoint(
                    self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y,
                    self.current_pose.pose.position.z
                )

            self.rate.sleep()

    def _send_setpoint(self, x, y, z):
        """发送位置设定点到飞控"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.setpoint_pub.publish(pose)

    # ============ 控制命令 ============

    def arm(self):
        """解锁"""
        req = CommandBoolRequest()
        req.value = True
        try:
            resp = self.arming_client(req)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"解锁失败: {e}")
            return False

    def disarm(self):
        """上锁"""
        req = CommandBoolRequest()
        req.value = False
        try:
            resp = self.arming_client(req)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"上锁失败: {e}")
            return False

    def set_mode(self, mode):
        """设置飞行模式"""
        req = SetModeRequest()
        req.custom_mode = mode
        try:
            resp = self.set_mode_client(req)
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr(f"模式切换失败: {e}")
            return False

    def takeoff(self, height=2.0):
        """
        起飞

        流程：
        1. 设置目标高度
        2. 预发送设定点
        3. 切换 OFFBOARD
        4. 解锁
        5. 开始飞行状态（后台线程持续发送设定点）
        """
        print(f"\n>>> 起飞到 {height}m...")

        if not self.current_state.connected:
            print("!!! 飞控未连接")
            return False

        # 设置起飞目标
        self.target_pose = (0.0, 0.0, height)
        self.is_flying = True

        # 预发送设定点（PX4 要求）
        print("    预发送设定点...")
        for _ in range(100):
            self._send_setpoint(0, 0, height)
            self.rate.sleep()

        # 切换模式和解锁
        last_request = rospy.Time.now()

        while not rospy.is_shutdown():
            if rospy.Time.now() - last_request > rospy.Duration(2.0):
                if self.current_state.mode != "OFFBOARD":
                    if self.set_mode("OFFBOARD"):
                        print("    切换到 OFFBOARD 模式")
                    last_request = rospy.Time.now()
                elif not self.current_state.armed:
                    if self.arm():
                        print("    已解锁")
                    last_request = rospy.Time.now()
                else:
                    # 已经是 OFFBOARD 且已解锁
                    break

            self.rate.sleep()

        # 等待到达高度
        print("    上升中...")
        while not rospy.is_shutdown():
            if self.current_pose:
                current_z = self.current_pose.pose.position.z
                if current_z > height - 0.3:
                    print(f">>> 起飞完成! 当前高度: {current_z:.1f}m")
                    return True
            self.rate.sleep()

        return False

    def land(self):
        """降落"""
        print("\n>>> 降落...")
        self.is_flying = False
        self.target_pose = None

        if self.set_mode("AUTO.LAND"):
            print("    切换到 AUTO.LAND 模式")
            return True
        return False

    def goto(self, x, y, z, use_planner=False):
        """
        飞到指定位置

        参数:
            x, y, z: 目标坐标
            use_planner: 是否使用路径规划器
        """
        if not self.is_flying:
            print("!!! 请先起飞 (按 t)")
            return

        print(f"\n>>> 飞往 ({x:.1f}, {y:.1f}, {z:.1f})")

        if use_planner:
            # 发送给路径规划器
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = z
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            print("    (使用路径规划)")
        else:
            # 直接设置目标（直线飞行）
            self.target_pose = (x, y, z)
            print("    (直线飞行)")

    def hover(self):
        """悬停在当前位置"""
        if self.current_pose:
            self.target_pose = (
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z
            )
            print(f"\n>>> 悬停在 ({self.target_pose[0]:.1f}, {self.target_pose[1]:.1f}, {self.target_pose[2]:.1f})")

    def get_distance_to_target(self):
        """计算到目标的距离"""
        if self.current_pose is None or self.target_pose is None:
            return None

        current = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])
        target = np.array(self.target_pose)
        return np.linalg.norm(current - target)

    # ============ 菜单界面 ============

    def print_status(self):
        """打印状态"""
        print("\n" + "-"*50)
        print("【状态】")
        print(f"  连接: {'✓' if self.current_state.connected else '✗'}")
        print(f"  解锁: {'✓' if self.current_state.armed else '✗'}")
        print(f"  模式: {self.current_state.mode}")
        print(f"  飞行: {'✓ 飞行中' if self.is_flying else '✗ 地面'}")

        if self.current_pose:
            p = self.current_pose.pose.position
            print(f"  位置: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")

        if self.target_pose:
            print(f"  目标: ({self.target_pose[0]:.2f}, {self.target_pose[1]:.2f}, {self.target_pose[2]:.2f})")
            dist = self.get_distance_to_target()
            if dist:
                print(f"  距离: {dist:.2f}m")

        print(f"  规划: {'✓ 使用路径规划' if self.use_planner else '✗ 直线飞行'}")
        print("-"*50)

    def print_menu(self):
        """打印菜单"""
        print("\n" + "="*50)
        print("      无人机控制面板 v2.0")
        print("="*50)

        print("\n【飞行控制】")
        print("  t. 起飞")
        print("  l. 降落")
        print("  h. 悬停")
        print("  p. 切换规划模式 (当前: {})".format("路径规划" if self.use_planner else "直线"))

        print("\n【预设目标】")
        for key, preset in self.presets.items():
            print(f"  {key}. {preset['name']:10} ({preset['x']}, {preset['y']}, {preset['z']})")

        print("\n【其他】")
        print("  c. 自定义坐标")
        print("  w. 航线飞行")
        print("  i. 查看状态")
        print("  q. 退出")
        print("-"*50)

    def run(self):
        """主循环"""
        print("\n" + "="*50)
        print("  无人机控制面板 v2.0")
        print("="*50)

        # 等待连接
        print("\n等待飞控连接...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()
        print("飞控已连接!")

        self.print_status()

        while not rospy.is_shutdown():
            self.print_menu()
            choice = input("请选择: ").strip().lower()

            if choice == 'q':
                if self.is_flying:
                    print("!!! 请先降落")
                else:
                    print("\n退出")
                    break
            elif choice == 't':
                height = input("高度 (默认 2.0): ").strip()
                height = float(height) if height else 2.0
                self.takeoff(height)
            elif choice == 'l':
                self.land()
            elif choice == 'h':
                self.hover()
            elif choice == 'p':
                self.use_planner = not self.use_planner
                print(f"\n>>> 切换到: {'路径规划' if self.use_planner else '直线飞行'}")
            elif choice == 'c':
                try:
                    x = float(input("  X: "))
                    y = float(input("  Y: "))
                    z = float(input("  Z: "))
                    self.goto(x, y, z, self.use_planner)
                except ValueError:
                    print("!!! 输入无效")
            elif choice == 'w':
                self._waypoint_mode()
            elif choice == 'i':
                self.print_status()
            elif choice in self.presets:
                p = self.presets[choice]
                self.goto(p['x'], p['y'], p['z'], self.use_planner)
            else:
                print("!!! 无效选择")

    def _waypoint_mode(self):
        """航线模式"""
        print("\n【航线】")
        print("  1. 正方形")
        print("  2. 三角形")
        print("  3. 自定义")

        choice = input("选择: ").strip()

        if choice == '1':
            waypoints = [(3,0,2), (3,3,2), (0,3,2), (0,0,2)]
        elif choice == '2':
            waypoints = [(4,0,2), (2,3,2), (0,0,2)]
        elif choice == '3':
            waypoints = []
            print("输入航点 (x y z)，输入 done 结束:")
            while True:
                inp = input(f"  点{len(waypoints)+1}: ").strip()
                if inp.lower() == 'done':
                    break
                try:
                    parts = inp.split()
                    if len(parts) == 3:
                        waypoints.append((float(parts[0]), float(parts[1]), float(parts[2])))
                except:
                    print("  格式错误")
        else:
            return

        if not waypoints:
            return

        print(f"\n将执行 {len(waypoints)} 个航点")
        for i, (x, y, z) in enumerate(waypoints):
            input(f"按 Enter 飞往航点 {i+1}: ({x}, {y}, {z})")
            self.goto(x, y, z, self.use_planner)

            # 等待到达
            print("    飞行中...")
            while not rospy.is_shutdown():
                dist = self.get_distance_to_target()
                if dist and dist < 0.5:
                    print(f"    到达航点 {i+1}")
                    break
                rospy.sleep(0.5)


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
