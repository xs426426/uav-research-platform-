#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D A* 路径规划算法
==================

核心概念：
- f(n) = g(n) + h(n)
- g(n): 起点到当前节点的实际代价
- h(n): 当前节点到终点的启发式估计

作者: UAV Research Platform
"""

import numpy as np
import heapq
from typing import List, Tuple, Optional, Set
import time


class Node:
    """A* 搜索节点"""

    def __init__(self, position: Tuple[int, int, int],
                 g: float = 0, h: float = 0,
                 parent: 'Node' = None):
        self.position = position  # (x, y, z)
        self.g = g  # 起点到当前节点的代价
        self.h = h  # 启发式估计
        self.f = g + h  # 总代价
        self.parent = parent

    def __lt__(self, other):
        """用于优先队列比较"""
        return self.f < other.f

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)


class AStarPlanner3D:
    """3D A* 路径规划器"""

    # 26 邻域方向（3D 空间中一个点的所有相邻点）
    DIRECTIONS = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                if dx != 0 or dy != 0 or dz != 0:
                    DIRECTIONS.append((dx, dy, dz))

    def __init__(self, grid_map: np.ndarray, resolution: float = 1.0):
        """
        初始化规划器

        参数:
            grid_map: 3D 栅格地图，0=自由空间，1=障碍物
            resolution: 栅格分辨率（米/格）
        """
        self.grid_map = grid_map
        self.resolution = resolution
        self.shape = grid_map.shape

    def heuristic(self, pos1: Tuple[int, int, int],
                  pos2: Tuple[int, int, int]) -> float:
        """
        启发式函数：欧几里得距离

        你也可以尝试其他启发式：
        - 曼哈顿距离: |x1-x2| + |y1-y2| + |z1-z2|
        - 对角距离
        """
        return np.sqrt((pos1[0] - pos2[0])**2 +
                       (pos1[1] - pos2[1])**2 +
                       (pos1[2] - pos2[2])**2)

    def is_valid(self, pos: Tuple[int, int, int]) -> bool:
        """检查位置是否有效（在地图内且不是障碍物）"""
        x, y, z = pos

        # 边界检查
        if not (0 <= x < self.shape[0] and
                0 <= y < self.shape[1] and
                0 <= z < self.shape[2]):
            return False

        # 障碍物检查
        if self.grid_map[x, y, z] == 1:
            return False

        return True

    def get_neighbors(self, node: Node) -> List[Tuple[Tuple[int, int, int], float]]:
        """
        获取节点的有效邻居

        返回: [(邻居位置, 移动代价), ...]
        """
        neighbors = []
        x, y, z = node.position

        for dx, dy, dz in self.DIRECTIONS:
            new_pos = (x + dx, y + dy, z + dz)

            if self.is_valid(new_pos):
                # 移动代价：对角移动代价更高
                cost = np.sqrt(dx**2 + dy**2 + dz**2)
                neighbors.append((new_pos, cost))

        return neighbors

    def reconstruct_path(self, node: Node) -> List[Tuple[int, int, int]]:
        """从终点回溯构建路径"""
        path = []
        current = node

        while current is not None:
            path.append(current.position)
            current = current.parent

        return path[::-1]  # 反转，从起点到终点

    def plan(self, start: Tuple[int, int, int],
             goal: Tuple[int, int, int],
             max_iterations: int = 100000) -> Optional[List[Tuple[int, int, int]]]:
        """
        执行 A* 搜索

        参数:
            start: 起点 (x, y, z)
            goal: 终点 (x, y, z)
            max_iterations: 最大迭代次数

        返回:
            路径列表，如果无解返回 None
        """
        # 验证起点终点
        if not self.is_valid(start):
            print(f"起点 {start} 无效（障碍物或超出边界）")
            return None
        if not self.is_valid(goal):
            print(f"终点 {goal} 无效（障碍物或超出边界）")
            return None

        # 初始化
        start_node = Node(start, g=0, h=self.heuristic(start, goal))

        # 优先队列（最小堆）
        open_list = [start_node]
        heapq.heapify(open_list)

        # 用于快速查找的字典
        open_dict = {start: start_node}
        closed_set: Set[Tuple[int, int, int]] = set()

        iterations = 0
        start_time = time.time()

        while open_list and iterations < max_iterations:
            iterations += 1

            # 取出 f 值最小的节点
            current = heapq.heappop(open_list)
            current_pos = current.position

            # 从 open_dict 中移除
            if current_pos in open_dict:
                del open_dict[current_pos]

            # 到达终点
            if current_pos == goal:
                elapsed = time.time() - start_time
                print(f"A* 搜索成功!")
                print(f"  迭代次数: {iterations}")
                print(f"  耗时: {elapsed:.3f}s")
                print(f"  路径长度: {len(self.reconstruct_path(current))}")
                return self.reconstruct_path(current)

            # 加入 closed_set
            closed_set.add(current_pos)

            # 遍历邻居
            for neighbor_pos, move_cost in self.get_neighbors(current):

                # 已经访问过
                if neighbor_pos in closed_set:
                    continue

                # 计算新的 g 值
                new_g = current.g + move_cost

                # 检查是否已在 open_list 中
                if neighbor_pos in open_dict:
                    existing = open_dict[neighbor_pos]
                    if new_g < existing.g:
                        # 找到更好的路径，更新
                        existing.g = new_g
                        existing.f = new_g + existing.h
                        existing.parent = current
                        heapq.heapify(open_list)  # 重新堆化
                else:
                    # 新节点
                    h = self.heuristic(neighbor_pos, goal)
                    neighbor_node = Node(neighbor_pos, g=new_g, h=h, parent=current)
                    heapq.heappush(open_list, neighbor_node)
                    open_dict[neighbor_pos] = neighbor_node

        print(f"A* 搜索失败: 达到最大迭代次数 {max_iterations}")
        return None

    def path_to_world(self, path: List[Tuple[int, int, int]]) -> List[Tuple[float, float, float]]:
        """将栅格路径转换为世界坐标"""
        return [(p[0] * self.resolution,
                 p[1] * self.resolution,
                 p[2] * self.resolution) for p in path]


def create_test_map(size: Tuple[int, int, int] = (50, 50, 20)) -> np.ndarray:
    """
    创建测试地图

    返回: 3D 栅格地图，0=自由，1=障碍
    """
    grid = np.zeros(size, dtype=np.int8)

    # 添加一些障碍物
    # 墙壁 1
    grid[20:30, 10:15, 0:15] = 1

    # 墙壁 2
    grid[10:15, 25:35, 0:12] = 1

    # 墙壁 3
    grid[30:40, 30:35, 5:18] = 1

    # 柱子
    grid[25:28, 25:28, 0:20] = 1

    return grid


def visualize_path(grid_map: np.ndarray,
                   path: List[Tuple[int, int, int]],
                   start: Tuple[int, int, int],
                   goal: Tuple[int, int, int]):
    """可视化路径（使用 matplotlib）"""
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
    except ImportError:
        print("需要安装 matplotlib: pip3 install matplotlib")
        return

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制障碍物
    obstacles = np.where(grid_map == 1)
    ax.scatter(obstacles[0], obstacles[1], obstacles[2],
               c='gray', alpha=0.3, s=20, label='Obstacles')

    # 绘制路径
    if path:
        path_array = np.array(path)
        ax.plot(path_array[:, 0], path_array[:, 1], path_array[:, 2],
                'b-', linewidth=2, label='Path')

    # 绘制起点终点
    ax.scatter(*start, c='green', s=200, marker='o', label='Start')
    ax.scatter(*goal, c='red', s=200, marker='*', label='Goal')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.set_title('3D A* Path Planning')

    plt.tight_layout()
    plt.savefig('astar_path.png', dpi=150)
    print("路径图已保存为 astar_path.png")
    plt.show()


def main():
    """测试 A* 算法"""
    print("=" * 50)
    print("3D A* 路径规划测试")
    print("=" * 50)

    # 创建地图
    grid_map = create_test_map((50, 50, 20))
    print(f"地图大小: {grid_map.shape}")
    print(f"障碍物占比: {np.sum(grid_map) / grid_map.size * 100:.1f}%")

    # 创建规划器
    planner = AStarPlanner3D(grid_map, resolution=0.5)

    # 设置起点终点
    start = (5, 5, 5)
    goal = (45, 45, 15)

    print(f"\n起点: {start}")
    print(f"终点: {goal}")
    print("\n开始搜索...")

    # 执行规划
    path = planner.plan(start, goal)

    if path:
        print(f"\n路径点数: {len(path)}")
        print(f"路径预览: {path[:3]} ... {path[-3:]}")

        # 转换为世界坐标
        world_path = planner.path_to_world(path)
        print(f"\n世界坐标路径预览:")
        for i, p in enumerate(world_path[:5]):
            print(f"  [{i}] ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})")

        # 可视化
        visualize_path(grid_map, path, start, goal)
    else:
        print("未找到路径!")


if __name__ == '__main__':
    main()
