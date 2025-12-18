#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交互式目标点发送工具

功能：
- 手动输入目标坐标
- 预设目标点快速选择
- 支持扩展更多功能

使用方法：
    python3 goal_sender.py
"""

import rospy
from geometry_msgs.msg import PoseStamped
import sys

class GoalSender:
    """目标点发送器"""

    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('goal_sender', anonymous=True)

        # 创建发布者
        # /move_base_simple/goal 是 ROS 导航的标准目标话题
        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal',
            PoseStamped,
            queue_size=10
        )

        # 等待发布者建立连接
        rospy.sleep(0.5)

        # 预设目标点（可扩展）
        self.presets = {
            '1': {'name': '前方悬停', 'x': 3.0, 'y': 0.0, 'z': 2.0},
            '2': {'name': '左前方', 'x': 3.0, 'y': 3.0, 'z': 2.0},
            '3': {'name': '右前方', 'x': 3.0, 'y': -3.0, 'z': 2.0},
            '4': {'name': '高空悬停', 'x': 0.0, 'y': 0.0, 'z': 5.0},
            '5': {'name': '绕圈起点', 'x': 5.0, 'y': 0.0, 'z': 3.0},
            '6': {'name': '远点测试', 'x': 10.0, 'y': 10.0, 'z': 3.0},
            '7': {'name': '返回原点', 'x': 0.0, 'y': 0.0, 'z': 2.0},
        }

    def send_goal(self, x, y, z):
        """
        发送目标点

        参数:
            x: 目标 X 坐标（前方为正，单位：米）
            y: 目标 Y 坐标（左方为正，单位：米）
            z: 目标 Z 坐标（高度，单位：米）
        """
        goal = PoseStamped()

        # 设置消息头
        goal.header.stamp = rospy.Time.now()  # 当前时间戳
        goal.header.frame_id = "map"          # 坐标系

        # 设置目标位置
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z

        # 设置目标姿态（四元数，w=1 表示无旋转）
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        # 发布目标
        self.goal_pub.publish(goal)
        print(f"\n>>> 已发送目标: x={x:.1f}, y={y:.1f}, z={z:.1f}")

    def print_menu(self):
        """打印菜单"""
        print("\n" + "="*50)
        print("        无人机目标点发送工具")
        print("="*50)
        print("\n【预设目标点】")
        for key, preset in self.presets.items():
            print(f"  {key}. {preset['name']:10} -> ({preset['x']}, {preset['y']}, {preset['z']})")
        print("\n【其他操作】")
        print("  c. 自定义坐标输入")
        print("  s. 连续发送多个点（航线）")
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
        """发送多个航点（简单航线）"""
        print("\n【航线模式】")
        print("预设航线：")
        print("  1. 正方形航线")
        print("  2. 三角形航线")
        print("  3. 自定义航点")

        choice = input("选择航线: ").strip()

        if choice == '1':
            # 正方形航线
            waypoints = [
                (3, 0, 2),
                (3, 3, 2),
                (0, 3, 2),
                (0, 0, 2),
            ]
        elif choice == '2':
            # 三角形航线
            waypoints = [
                (4, 0, 2),
                (2, 3, 2),
                (0, 0, 2),
            ]
        elif choice == '3':
            # 自定义航点
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
        print("提示：每个点会间隔发送，观察无人机到达后再发下一个")

        for i, (x, y, z) in enumerate(waypoints):
            input(f"\n按 Enter 发送航点 {i+1}/{len(waypoints)}: ({x}, {y}, {z})")
            self.send_goal(x, y, z)

    def run(self):
        """主循环"""
        print("\n" + "="*50)
        print("  目标发送器已启动")
        print("  确保 path_planner_node.py 正在运行")
        print("="*50)

        while not rospy.is_shutdown():
            self.print_menu()
            choice = input("请选择: ").strip().lower()

            if choice == 'q':
                print("\n退出程序")
                break
            elif choice == 'c':
                result = self.get_custom_goal()
                if result:
                    self.send_goal(*result)
            elif choice == 's':
                self.send_waypoints()
            elif choice in self.presets:
                preset = self.presets[choice]
                print(f"\n选择: {preset['name']}")
                self.send_goal(preset['x'], preset['y'], preset['z'])
            else:
                print("\n!!! 无效选择，请重试")


def main():
    try:
        sender = GoalSender()
        sender.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n\n用户中断")


if __name__ == '__main__':
    main()
