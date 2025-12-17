#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D RRT* 路径规划算法
====================

RRT* 是 RRT 的改进版本，具有渐进最优性。

核心改进：
1. 选择代价最小的父节点
2. rewire: 检查新节点是否能作为附近节点的更优父节点

作者: UAV Research Platform
"""

import numpy as np
from typing import List, Tuple, Optional
import time


class RRTNode:
    """RRT 树节点"""

    def __init__(self, position: np.ndarray, parent: 'RRTNode' = None):
        self.position = position  # [x, y, z]
        self.parent = parent
        self.cost = 0.0  # 从根节点到此节点的代价
        self.children = []


class RRTStarPlanner3D:
    """3D RRT* 路径规划器"""

    def __init__(self,
                 x_range: Tuple[float, float],
                 y_range: Tuple[float, float],
                 z_range: Tuple[float, float],
                 obstacles: List[Tuple[np.ndarray, np.ndarray]] = None,
                 step_size: float = 1.0,
                 goal_sample_rate: float = 0.1,
                 search_radius: float = 3.0):
        """
        初始化 RRT* 规划器

        参数:
            x_range, y_range, z_range: 搜索空间范围
            obstacles: 障碍物列表 [(min_corner, max_corner), ...]
            step_size: 每步扩展的距离
            goal_sample_rate: 采样目标点的概率
            search_radius: rewire 搜索半径
        """
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        self.obstacles = obstacles or []
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius

        self.nodes: List[RRTNode] = []
        self.start = None
        self.goal = None

    def distance(self, p1: np.ndarray, p2: np.ndarray) -> float:
        """计算两点间的欧几里得距离"""
        return np.linalg.norm(p1 - p2)

    def random_sample(self) -> np.ndarray:
        """
        随机采样

        以 goal_sample_rate 的概率采样目标点，
        否则在搜索空间中均匀随机采样
        """
        if np.random.random() < self.goal_sample_rate:
            return self.goal.copy()
        else:
            return np.array([
                np.random.uniform(self.x_range[0], self.x_range[1]),
                np.random.uniform(self.y_range[0], self.y_range[1]),
                np.random.uniform(self.z_range[0], self.z_range[1])
            ])

    def nearest_node(self, point: np.ndarray) -> RRTNode:
        """找到树中距离给定点最近的节点"""
        min_dist = float('inf')
        nearest = None

        for node in self.nodes:
            dist = self.distance(node.position, point)
            if dist < min_dist:
                min_dist = dist
                nearest = node

        return nearest

    def steer(self, from_pos: np.ndarray, to_pos: np.ndarray) -> np.ndarray:
        """
        从 from_pos 向 to_pos 方向扩展 step_size 距离

        如果距离小于 step_size，直接返回 to_pos
        """
        direction = to_pos - from_pos
        dist = np.linalg.norm(direction)

        if dist < self.step_size:
            return to_pos.copy()
        else:
            direction = direction / dist  # 单位向量
            return from_pos + direction * self.step_size

    def is_collision_free(self, p1: np.ndarray, p2: np.ndarray,
                          num_checks: int = 10) -> bool:
        """
        检查从 p1 到 p2 的路径是否无碰撞

        通过在路径上均匀采样多个点来检查
        """
        for i in range(num_checks + 1):
            t = i / num_checks
            point = p1 + t * (p2 - p1)

            for obs_min, obs_max in self.obstacles:
                # AABB (轴对齐包围盒) 碰撞检测
                if np.all(point >= obs_min) and np.all(point <= obs_max):
                    return False

        return True

    def near_nodes(self, point: np.ndarray, radius: float) -> List[RRTNode]:
        """找到给定半径内的所有节点"""
        near = []
        for node in self.nodes:
            if self.distance(node.position, point) <= radius:
                near.append(node)
        return near

    def choose_parent(self, new_pos: np.ndarray,
                      near_nodes: List[RRTNode]) -> Tuple[RRTNode, float]:
        """
        从附近节点中选择代价最小的父节点

        这是 RRT* 相对于 RRT 的关键改进之一
        """
        min_cost = float('inf')
        best_parent = None

        for node in near_nodes:
            if self.is_collision_free(node.position, new_pos):
                cost = node.cost + self.distance(node.position, new_pos)
                if cost < min_cost:
                    min_cost = cost
                    best_parent = node

        return best_parent, min_cost

    def rewire(self, new_node: RRTNode, near_nodes: List[RRTNode]):
        """
        重新连接（rewire）操作

        检查新节点是否能成为附近节点的更优父节点
        这是 RRT* 渐进最优性的关键
        """
        for node in near_nodes:
            if node == new_node.parent:
                continue

            new_cost = new_node.cost + self.distance(new_node.position, node.position)

            if new_cost < node.cost:
                if self.is_collision_free(new_node.position, node.position):
                    # 更新父节点
                    if node.parent:
                        node.parent.children.remove(node)
                    node.parent = new_node
                    node.cost = new_cost
                    new_node.children.append(node)

                    # 递归更新子节点的代价
                    self._update_children_cost(node)

    def _update_children_cost(self, node: RRTNode):
        """递归更新子节点的代价"""
        for child in node.children:
            child.cost = node.cost + self.distance(node.position, child.position)
            self._update_children_cost(child)

    def plan(self,
             start: np.ndarray,
             goal: np.ndarray,
             max_iterations: int = 5000,
             goal_tolerance: float = 1.0) -> Optional[List[np.ndarray]]:
        """
        执行 RRT* 规划

        参数:
            start: 起点 [x, y, z]
            goal: 终点 [x, y, z]
            max_iterations: 最大迭代次数
            goal_tolerance: 到达目标的容差

        返回:
            路径点列表，如果失败返回 None
        """
        self.start = np.array(start)
        self.goal = np.array(goal)

        # 初始化树
        start_node = RRTNode(self.start)
        start_node.cost = 0.0
        self.nodes = [start_node]

        best_goal_node = None
        best_goal_cost = float('inf')

        start_time = time.time()

        for i in range(max_iterations):
            # 1. 随机采样
            sample = self.random_sample()

            # 2. 找到最近节点
            nearest = self.nearest_node(sample)

            # 3. 向采样点方向扩展
            new_pos = self.steer(nearest.position, sample)

            # 4. 碰撞检测
            if not self.is_collision_free(nearest.position, new_pos):
                continue

            # 5. 找到附近节点
            near = self.near_nodes(new_pos, self.search_radius)
            if not near:
                near = [nearest]

            # 6. 选择最优父节点 (RRT* 改进)
            best_parent, min_cost = self.choose_parent(new_pos, near)
            if best_parent is None:
                continue

            # 7. 创建新节点
            new_node = RRTNode(new_pos, parent=best_parent)
            new_node.cost = min_cost
            best_parent.children.append(new_node)
            self.nodes.append(new_node)

            # 8. rewire 操作 (RRT* 改进)
            self.rewire(new_node, near)

            # 9. 检查是否到达目标
            dist_to_goal = self.distance(new_pos, self.goal)
            if dist_to_goal < goal_tolerance:
                if new_node.cost < best_goal_cost:
                    best_goal_node = new_node
                    best_goal_cost = new_node.cost
                    print(f"  找到路径! 代价: {best_goal_cost:.2f}, 迭代: {i}")

            # 进度显示
            if (i + 1) % 1000 == 0:
                print(f"  迭代 {i + 1}/{max_iterations}, 节点数: {len(self.nodes)}")

        elapsed = time.time() - start_time

        if best_goal_node:
            path = self._extract_path(best_goal_node)
            print(f"\nRRT* 搜索完成!")
            print(f"  总迭代: {max_iterations}")
            print(f"  耗时: {elapsed:.2f}s")
            print(f"  最终代价: {best_goal_cost:.2f}")
            print(f"  路径点数: {len(path)}")
            return path
        else:
            print(f"\nRRT* 搜索失败: 未能到达目标")
            return None

    def _extract_path(self, goal_node: RRTNode) -> List[np.ndarray]:
        """从目标节点回溯提取路径"""
        path = [self.goal.copy()]  # 添加精确目标点

        node = goal_node
        while node is not None:
            path.append(node.position.copy())
            node = node.parent

        return path[::-1]  # 反转


def create_obstacles() -> List[Tuple[np.ndarray, np.ndarray]]:
    """创建测试障碍物"""
    obstacles = [
        # (min_corner, max_corner)
        (np.array([8, 0, 0]), np.array([12, 20, 15])),   # 墙壁 1
        (np.array([8, 30, 0]), np.array([12, 50, 15])),  # 墙壁 2
        (np.array([25, 15, 0]), np.array([30, 35, 12])), # 墙壁 3
        (np.array([15, 22, 0]), np.array([18, 28, 20])), # 柱子
    ]
    return obstacles


def visualize_rrt(planner: RRTStarPlanner3D,
                  path: List[np.ndarray],
                  start: np.ndarray,
                  goal: np.ndarray):
    """可视化 RRT 树和路径"""
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ImportError:
        print("需要安装 matplotlib: pip3 install matplotlib")
        return

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制障碍物
    for obs_min, obs_max in planner.obstacles:
        # 创建立方体的8个顶点
        vertices = [
            [obs_min[0], obs_min[1], obs_min[2]],
            [obs_max[0], obs_min[1], obs_min[2]],
            [obs_max[0], obs_max[1], obs_min[2]],
            [obs_min[0], obs_max[1], obs_min[2]],
            [obs_min[0], obs_min[1], obs_max[2]],
            [obs_max[0], obs_min[1], obs_max[2]],
            [obs_max[0], obs_max[1], obs_max[2]],
            [obs_min[0], obs_max[1], obs_max[2]],
        ]
        # 定义6个面
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],
            [vertices[4], vertices[5], vertices[6], vertices[7]],
            [vertices[0], vertices[1], vertices[5], vertices[4]],
            [vertices[2], vertices[3], vertices[7], vertices[6]],
            [vertices[0], vertices[3], vertices[7], vertices[4]],
            [vertices[1], vertices[2], vertices[6], vertices[5]],
        ]
        ax.add_collection3d(Poly3DCollection(faces, alpha=0.3,
                                              facecolor='gray',
                                              edgecolor='darkgray'))

    # 绘制 RRT 树（采样显示，避免太密集）
    sample_rate = max(1, len(planner.nodes) // 500)
    for i, node in enumerate(planner.nodes):
        if i % sample_rate == 0 and node.parent:
            xs = [node.parent.position[0], node.position[0]]
            ys = [node.parent.position[1], node.position[1]]
            zs = [node.parent.position[2], node.position[2]]
            ax.plot(xs, ys, zs, 'c-', alpha=0.2, linewidth=0.5)

    # 绘制路径
    if path:
        path_array = np.array(path)
        ax.plot(path_array[:, 0], path_array[:, 1], path_array[:, 2],
                'b-', linewidth=3, label='Path')
        ax.scatter(path_array[:, 0], path_array[:, 1], path_array[:, 2],
                   c='blue', s=30)

    # 绘制起点终点
    ax.scatter(*start, c='green', s=300, marker='o', label='Start')
    ax.scatter(*goal, c='red', s=300, marker='*', label='Goal')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_xlim(planner.x_range)
    ax.set_ylim(planner.y_range)
    ax.set_zlim(planner.z_range)
    ax.legend()
    ax.set_title(f'RRT* Path Planning (Nodes: {len(planner.nodes)})')

    plt.tight_layout()
    plt.savefig('rrt_star_path.png', dpi=150)
    print("路径图已保存为 rrt_star_path.png")
    plt.show()


def main():
    """测试 RRT* 算法"""
    print("=" * 50)
    print("3D RRT* 路径规划测试")
    print("=" * 50)

    # 创建障碍物
    obstacles = create_obstacles()
    print(f"障碍物数量: {len(obstacles)}")

    # 创建规划器
    planner = RRTStarPlanner3D(
        x_range=(0, 50),
        y_range=(0, 50),
        z_range=(0, 20),
        obstacles=obstacles,
        step_size=2.0,
        goal_sample_rate=0.1,
        search_radius=5.0
    )

    # 设置起点终点
    start = np.array([2, 2, 5])
    goal = np.array([45, 45, 10])

    print(f"\n起点: {start}")
    print(f"终点: {goal}")
    print("\n开始 RRT* 搜索...")

    # 执行规划
    path = planner.plan(start, goal, max_iterations=3000, goal_tolerance=2.0)

    if path:
        print(f"\n路径预览:")
        for i, p in enumerate(path[:5]):
            print(f"  [{i}] ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})")
        if len(path) > 5:
            print(f"  ... ({len(path) - 5} more points)")

        # 可视化
        visualize_rrt(planner, path, start, goal)
    else:
        print("未找到路径!")


if __name__ == '__main__':
    main()
