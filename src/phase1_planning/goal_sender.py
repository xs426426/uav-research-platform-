#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机控制面板 v3.0 - 简洁可靠版

核心改进：
1. 非阻塞设计 - 所有操作立即返回
2. 后台线程持续发送设定点
3. 随时可以更改目标
4. 简化逻辑，减少 bug

使用：python3 goal_sender.py
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import threading
import time


class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)

        # 状态
        self.state = State()
        self.pose = None
        self.target = None  # (x, y, z) 目标位置
        self.running = True

        # 订阅
        rospy.Subscriber('/mavros/state', State, lambda m: setattr(self, 'state', m))
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, lambda m: setattr(self, 'pose', m))

        # 发布
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # 服务
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # 预设点
        self.presets = {
            '1': ('前方3m', 3, 0, 2),
            '2': ('左前方', 3, 3, 2),
            '3': ('右前方', 3, -3, 2),
            '4': ('高空5m', 0, 0, 5),
            '5': ('远点', 10, 0, 3),
            '6': ('原点', 0, 0, 2),
        }

        # 启动后台线程
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

        print("控制器就绪")

    def _loop(self):
        """后台循环 - 20Hz 持续发送设定点"""
        rate = rospy.Rate(20)
        while self.running and not rospy.is_shutdown():
            if self.target:
                self._publish_setpoint(*self.target)
            rate.sleep()

    def _publish_setpoint(self, x, y, z):
        """发送设定点"""
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def takeoff(self, h=2.0):
        """起飞"""
        print(f"\n起飞到 {h}m...")

        # 1. 设置目标并开始发送
        self.target = (0, 0, h)

        # 2. 预发送 2 秒（PX4 要求）
        print("  预发送设定点...")
        time.sleep(2)

        # 3. 切换 OFFBOARD
        try:
            self.mode_srv(custom_mode="OFFBOARD")
            print("  OFFBOARD 模式")
        except:
            print("  !!! 模式切换失败")
            return

        # 4. 解锁
        try:
            self.arm_srv(True)
            print("  已解锁")
        except:
            print("  !!! 解锁失败")
            return

        print("  起飞中... (后台执行)")

    def land(self):
        """降落"""
        print("\n降落...")
        try:
            self.mode_srv(custom_mode="AUTO.LAND")
            self.target = None  # 停止发送设定点
            print("  降落中...")
        except:
            print("  !!! 降落失败")

    def goto(self, x, y, z):
        """飞到目标点（非阻塞）"""
        print(f"\n飞往 ({x}, {y}, {z})")
        self.target = (x, y, z)

    def goto_planner(self, x, y, z):
        """通过路径规划器飞到目标点"""
        print(f"\n规划飞往 ({x}, {y}, {z})")
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def status(self):
        """显示状态"""
        print("\n--- 状态 ---")
        print(f"连接: {'✓' if self.state.connected else '✗'}")
        print(f"解锁: {'✓' if self.state.armed else '✗'}")
        print(f"模式: {self.state.mode}")
        if self.pose:
            p = self.pose.pose.position
            print(f"位置: ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})")
        if self.target:
            print(f"目标: {self.target}")
        print("------------")

    def run(self):
        """主循环"""
        # 等待连接
        print("等待飞控...")
        while not self.state.connected and not rospy.is_shutdown():
            time.sleep(0.1)
        print("已连接!\n")

        use_planner = False

        while not rospy.is_shutdown():
            print("\n===== 无人机控制 =====")
            print("【控制】t=起飞 l=降落 h=悬停")
            print("【目标】1-6=预设点 c=自定义 w=航线")
            print(f"【模式】p=切换 (当前:{'规划' if use_planner else '直线'})")
            print("【其他】i=状态 q=退出")
            print("=" * 22)

            for k, v in self.presets.items():
                print(f"  {k}. {v[0]} ({v[1]},{v[2]},{v[3]})")

            cmd = input("\n> ").strip().lower()

            if cmd == 'q':
                self.running = False
                break
            elif cmd == 't':
                h = input("高度(默认2): ").strip()
                self.takeoff(float(h) if h else 2.0)
            elif cmd == 'l':
                self.land()
            elif cmd == 'h':
                if self.pose:
                    p = self.pose.pose.position
                    self.target = (p.x, p.y, p.z)
                    print(f"悬停在 ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})")
            elif cmd == 'p':
                use_planner = not use_planner
                print(f"切换到: {'路径规划' if use_planner else '直线飞行'}")
            elif cmd == 'i':
                self.status()
            elif cmd == 'c':
                try:
                    x = float(input("X: "))
                    y = float(input("Y: "))
                    z = float(input("Z: "))
                    if use_planner:
                        self.goto_planner(x, y, z)
                    else:
                        self.goto(x, y, z)
                except:
                    print("输入错误")
            elif cmd == 'w':
                self._waypoints(use_planner)
            elif cmd in self.presets:
                _, x, y, z = self.presets[cmd]
                if use_planner:
                    self.goto_planner(x, y, z)
                else:
                    self.goto(x, y, z)

    def _waypoints(self, use_planner):
        """航线模式"""
        print("\n1=正方形 2=三角形 3=自定义")
        c = input("选择: ").strip()

        if c == '1':
            pts = [(3,0,2), (3,3,2), (0,3,2), (0,0,2)]
        elif c == '2':
            pts = [(4,0,2), (2,3,2), (0,0,2)]
        elif c == '3':
            pts = []
            print("输入 x y z，输入 done 结束")
            while True:
                s = input(f"点{len(pts)+1}: ").strip()
                if s == 'done':
                    break
                try:
                    p = s.split()
                    pts.append((float(p[0]), float(p[1]), float(p[2])))
                except:
                    pass
        else:
            return

        print(f"\n共 {len(pts)} 个航点，按 Enter 发送下一个")
        for i, (x, y, z) in enumerate(pts):
            input(f"发送航点 {i+1}: ({x},{y},{z})")
            if use_planner:
                self.goto_planner(x, y, z)
            else:
                self.goto(x, y, z)


if __name__ == '__main__':
    try:
        DroneController().run()
    except KeyboardInterrupt:
        print("\n退出")
