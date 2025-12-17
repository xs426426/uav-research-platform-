#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路径规划 ROS 节点
=================

将路径规划算法与 PX4/MAVROS 集成，实现真正的自主避障飞行。

功能：
1. 接收目标点
2. 使用 A* 或 RRT* 规划路径
3. 发送航点到 PX4

作者: UAV Research Platform
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from typing import List, Tuple, Optional
import threading

# 导入我们的规划算法
from astar_3d import AStarPlanner3D
from rrt_star_3d import RRTStarPlanner3D


class PathPlannerNode:
    """路径规划 ROS 节点"""

    def __init__(self):
        rospy.init_node('path_planner_node', anonymous=True)

        # ============ 参数 ============
        self.map_size = rospy.get_param('~map_size', [50, 50, 20])
        self.resolution = rospy.get_param('~resolution', 0.5)
        self.planner_type = rospy.get_param('~planner', 'astar')  # 'astar' or 'rrt_star'

        # ============ 状态 ============
        self.current_pose = None
        self.current_state = State()
        self.grid_map = None
        self.is_planning = False

        # ============ 订阅者 ============
        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            self.pose_callback
        )

        self.state_sub = rospy.Subscriber(
            '/mavros/state',
            State,
            self.state_callback
        )

        self.goal_sub = rospy.Subscriber(
            '/move_base_simple/goal',  # 可以用 RViz 的 2D Nav Goal 发送
            PoseStamped,
            self.goal_callback
        )

        # ============ 发布者 ============
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )

        self.path_pub = rospy.Publisher(
            '/planned_path',
            Path,
            queue_size=1
        )

        self.marker_pub = rospy.Publisher(
            '/planning_markers',
            MarkerArray,
            queue_size=1
        )

        # ============ 服务客户端 ============
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # ============ 初始化地图 ============
        self._init_map()

        # ============ 控制循环 ============
        self.rate = rospy.Rate(20)
        self.current_path = []
        self.current_waypoint_idx = 0
        self.waypoint_tolerance = 0.5  # 到达航点的容差

        rospy.loginfo("路径规划节点初始化完成")
        rospy.loginfo(f"  规划器类型: {self.planner_type}")
        rospy.loginfo(f"  地图大小: {self.map_size}")
        rospy.loginfo(f"  分辨率: {self.resolution} m")

    def _init_map(self):
        """初始化栅格地图"""
        grid_shape = tuple(int(s / self.resolution) for s in self.map_size)
        self.grid_map = np.zeros(grid_shape, dtype=np.int8)

        # 添加一些测试障碍物（实际应用中应该从传感器获取）
        # 这里创建几个立方体障碍物
        self._add_obstacle((5, 5, 0), (8, 8, 10))
        self._add_obstacle((12, 2, 0), (15, 15, 8))
        self._add_obstacle((3, 15, 0), (10, 18, 12))

        rospy.loginfo(f"  地图栅格大小: {self.grid_map.shape}")
        rospy.loginfo(f"  障碍物占比: {np.sum(self.grid_map) / self.grid_map.size * 100:.1f}%")

    def _add_obstacle(self, min_corner: Tuple[float, float, float],
                      max_corner: Tuple[float, float, float]):
        """添加障碍物到栅格地图"""
        # 转换为栅格坐标
        min_idx = tuple(int(c / self.resolution) for c in min_corner)
        max_idx = tuple(int(c / self.resolution) for c in max_corner)

        # 确保在边界内
        min_idx = tuple(max(0, i) for i in min_idx)
        max_idx = tuple(min(s, i) for s, i in zip(self.grid_map.shape, max_idx))

        self.grid_map[min_idx[0]:max_idx[0],
                      min_idx[1]:max_idx[1],
                      min_idx[2]:max_idx[2]] = 1

    def pose_callback(self, msg):
        """位置回调"""
        self.current_pose = msg

    def state_callback(self, msg):
        """状态回调"""
        self.current_state = msg

    def goal_callback(self, msg):
        """接收目标点，开始规划"""
        if self.is_planning:
            rospy.logwarn("正在规划中，忽略新目标")
            return

        if self.current_pose is None:
            rospy.logwarn("当前位置未知，无法规划")
            return

        goal = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z if msg.pose.position.z > 0 else 2.0  # 默认高度 2m
        ])

        rospy.loginfo(f"收到目标点: ({goal[0]:.1f}, {goal[1]:.1f}, {goal[2]:.1f})")

        # 在新线程中规划（避免阻塞）
        threading.Thread(target=self._plan_path, args=(goal,)).start()

    def _plan_path(self, goal: np.ndarray):
        """执行路径规划"""
        self.is_planning = True

        start = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])

        rospy.loginfo(f"开始规划: {start} -> {goal}")
        rospy.loginfo(f"使用 {self.planner_type} 算法")

        path = None

        if self.planner_type == 'astar':
            path = self._plan_astar(start, goal)
        elif self.planner_type == 'rrt_star':
            path = self._plan_rrt_star(start, goal)
        else:
            rospy.logerr(f"未知规划器类型: {self.planner_type}")

        if path:
            rospy.loginfo(f"规划成功! 路径点数: {len(path)}")
            self.current_path = path
            self.current_waypoint_idx = 0
            self._publish_path(path)
            self._publish_markers(path)
        else:
            rospy.logerr("规划失败!")

        self.is_planning = False

    def _plan_astar(self, start: np.ndarray, goal: np.ndarray) -> Optional[List[np.ndarray]]:
        """使用 A* 规划"""
        # 转换为栅格坐标
        start_grid = tuple(int(c / self.resolution) for c in start)
        goal_grid = tuple(int(c / self.resolution) for c in goal)

        planner = AStarPlanner3D(self.grid_map, self.resolution)
        path_grid = planner.plan(start_grid, goal_grid)

        if path_grid:
            # 转换回世界坐标
            return [np.array(p) * self.resolution for p in path_grid]
        return None

    def _plan_rrt_star(self, start: np.ndarray, goal: np.ndarray) -> Optional[List[np.ndarray]]:
        """使用 RRT* 规划"""
        # 从栅格地图提取障碍物
        obstacles = self._extract_obstacles()

        planner = RRTStarPlanner3D(
            x_range=(0, self.map_size[0]),
            y_range=(0, self.map_size[1]),
            z_range=(0, self.map_size[2]),
            obstacles=obstacles,
            step_size=1.0,
            goal_sample_rate=0.15,
            search_radius=3.0
        )

        return planner.plan(start, goal, max_iterations=2000, goal_tolerance=1.0)

    def _extract_obstacles(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """从栅格地图提取障碍物包围盒"""
        # 简化版：返回预定义的障碍物
        # 实际应用中应该从栅格地图中提取连通区域
        return [
            (np.array([5, 5, 0]) , np.array([8, 8, 10])),
            (np.array([12, 2, 0]), np.array([15, 15, 8])),
            (np.array([3, 15, 0]), np.array([10, 18, 12])),
        ]

    def _publish_path(self, path: List[np.ndarray]):
        """发布路径到 ROS 话题（用于 RViz 可视化）"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for p in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def _publish_markers(self, path: List[np.ndarray]):
        """发布可视化标记"""
        markers = MarkerArray()

        # 路径标记
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = rospy.Time.now()
        path_marker.ns = "path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.1
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0

        for p in path:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            point.z = p[2]
            path_marker.points.append(point)

        markers.markers.append(path_marker)

        # 航点标记
        for i, p in enumerate(path):
            wp_marker = Marker()
            wp_marker.header.frame_id = "map"
            wp_marker.header.stamp = rospy.Time.now()
            wp_marker.ns = "waypoints"
            wp_marker.id = i + 1
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = p[0]
            wp_marker.pose.position.y = p[1]
            wp_marker.pose.position.z = p[2]
            wp_marker.scale.x = 0.3
            wp_marker.scale.y = 0.3
            wp_marker.scale.z = 0.3
            wp_marker.color.r = 1.0
            wp_marker.color.g = 0.5
            wp_marker.color.b = 0.0
            wp_marker.color.a = 0.8
            markers.markers.append(wp_marker)

        self.marker_pub.publish(markers)

    def send_setpoint(self, x: float, y: float, z: float):
        """发送位置设定点"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.setpoint_pub.publish(pose)

    def execute_path(self):
        """执行路径跟踪"""
        if not self.current_path or self.current_waypoint_idx >= len(self.current_path):
            return

        # 当前目标航点
        target = self.current_path[self.current_waypoint_idx]

        # 发送设定点
        self.send_setpoint(target[0], target[1], target[2])

        # 检查是否到达
        if self.current_pose:
            current = np.array([
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z
            ])
            dist = np.linalg.norm(current - target)

            if dist < self.waypoint_tolerance:
                rospy.loginfo(f"到达航点 {self.current_waypoint_idx + 1}/{len(self.current_path)}")
                self.current_waypoint_idx += 1

                if self.current_waypoint_idx >= len(self.current_path):
                    rospy.loginfo("路径执行完成!")

    def run(self):
        """主循环"""
        rospy.loginfo("等待飞控连接...")

        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

        rospy.loginfo("飞控已连接!")
        rospy.loginfo("等待目标点... (可以用 RViz 的 2D Nav Goal 发送)")

        while not rospy.is_shutdown():
            self.execute_path()
            self.rate.sleep()


def main():
    try:
        node = PathPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
