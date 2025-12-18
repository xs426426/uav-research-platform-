# 01. A* 搜索算法

## 一、算法本质

A* 是一种**启发式搜索算法**，结合了 Dijkstra 的最短路径保证和贪心搜索的效率。

```
核心思想：在搜索过程中，优先探索"看起来最有希望"的方向
```

## 二、核心公式

```
f(n) = g(n) + h(n)

其中：
┌──────────────────────────────────────────────────────────────┐
│  f(n) = 节点 n 的总代价估计（用于决定搜索顺序）              │
│  g(n) = 从起点到节点 n 的实际代价（已知、精确）              │
│  h(n) = 从节点 n 到终点的估计代价（启发式函数）              │
└──────────────────────────────────────────────────────────────┘
```

### 各部分的含义

```
       起点 ──────────────── n ─ ─ ─ ─ ─ ─ ─ 终点
              │                    │
              │← g(n): 已走过的路 →│← h(n): 预估剩余路 →│
              │     (精确值)       │     (估计值)       │
              │                    │                    │
              └────────────────────┴────────────────────┘
                              f(n): 总代价估计
```

## 三、算法流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                         A* 算法流程                                  │
└─────────────────────────────────────────────────────────────────────┘

1. 初始化
   ┌─────────────────┐
   │  Open List      │ ← 放入起点（待探索的节点）
   │  [起点]         │
   └─────────────────┘
   ┌─────────────────┐
   │  Closed List    │ ← 空（已探索的节点）
   │  []             │
   └─────────────────┘

2. 主循环
   ┌────────────────────────────────────────────────────────────────┐
   │  while Open List 不为空:                                       │
   │      │                                                         │
   │      ├─► 从 Open List 取出 f 值最小的节点 current              │
   │      │                                                         │
   │      ├─► 如果 current == 终点:                                 │
   │      │       回溯路径，返回结果 ✓                              │
   │      │                                                         │
   │      ├─► 将 current 加入 Closed List                           │
   │      │                                                         │
   │      └─► 遍历 current 的所有邻居 neighbor:                     │
   │              │                                                 │
   │              ├─► 如果 neighbor 在 Closed List: 跳过            │
   │              │                                                 │
   │              ├─► 计算新的 g 值: new_g = g(current) + cost      │
   │              │                                                 │
   │              └─► 如果 neighbor 不在 Open List                  │
   │                  或 new_g < g(neighbor):                       │
   │                      更新 neighbor 的 g, h, f 值               │
   │                      设置 neighbor 的父节点为 current          │
   │                      将 neighbor 加入 Open List                │
   └────────────────────────────────────────────────────────────────┘
```

## 四、启发式函数

### 常用启发式函数

```python
# 3D 空间中常用的启发式函数

# 1. 欧几里得距离（直线距离）
def euclidean(n, goal):
    return sqrt((n.x - goal.x)² + (n.y - goal.y)² + (n.z - goal.z)²)

# 2. 曼哈顿距离（格子距离，只能横竖走）
def manhattan(n, goal):
    return |n.x - goal.x| + |n.y - goal.y| + |n.z - goal.z|

# 3. 对角线距离（可以斜着走）
def diagonal(n, goal):
    dx = |n.x - goal.x|
    dy = |n.y - goal.y|
    dz = |n.z - goal.z|
    return max(dx, dy, dz) + (sqrt(2)-1) * second_max + (sqrt(3)-sqrt(2)) * min
```

### 启发式函数的要求

```
┌──────────────────────────────────────────────────────────────────────┐
│  可采纳性 (Admissibility)                                            │
│  ──────────────────────────                                          │
│  h(n) ≤ 实际最短距离                                                 │
│                                                                      │
│  含义：启发式函数不能高估到终点的距离                                │
│  作用：保证找到的路径是最优的                                        │
│                                                                      │
│  例如：                                                              │
│  - 直线距离永远 ≤ 实际路径距离（因为直线最短）                       │
│  - 所以欧几里得距离是可采纳的                                        │
└──────────────────────────────────────────────────────────────────────┘
```

## 五、3D 邻居定义

```python
# 26-邻域：当前点周围所有相邻的格子
# 想象一个 3x3x3 的魔方，中心点有 26 个邻居

NEIGHBORS_26 = [
    # 6 个面邻居（距离 = 1）
    (1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1),

    # 12 个边邻居（距离 = √2 ≈ 1.414）
    (1,1,0), (1,-1,0), (-1,1,0), (-1,-1,0),
    (1,0,1), (1,0,-1), (-1,0,1), (-1,0,-1),
    (0,1,1), (0,1,-1), (0,-1,1), (0,-1,-1),

    # 8 个角邻居（距离 = √3 ≈ 1.732）
    (1,1,1), (1,1,-1), (1,-1,1), (1,-1,-1),
    (-1,1,1), (-1,1,-1), (-1,-1,1), (-1,-1,-1)
]

# 图示（2D 简化版，8-邻域）：
#
#   ↖ ↑ ↗
#   ← ● →
#   ↙ ↓ ↘
```

## 六、代码实现

```python
import heapq
import numpy as np

class Node:
    """A* 节点"""
    def __init__(self, position, g=float('inf'), h=0, parent=None):
        self.position = position  # (x, y, z) 坐标
        self.g = g                # 从起点到当前的代价
        self.h = h                # 启发式估计（到终点的估计）
        self.parent = parent      # 父节点（用于回溯路径）

    @property
    def f(self):
        return self.g + self.h

    def __lt__(self, other):
        return self.f < other.f   # 用于优先队列比较

class AStar3D:
    """3D A* 路径规划"""

    def __init__(self, grid_map, resolution=0.5):
        """
        参数:
            grid_map: 3D numpy 数组，0=自由，1=障碍
            resolution: 每个格子代表的实际距离(米)
        """
        self.grid = grid_map
        self.resolution = resolution

    def plan(self, start, goal):
        """
        规划路径

        参数:
            start: 起点 (x, y, z) 世界坐标
            goal:  终点 (x, y, z) 世界坐标

        返回:
            路径点列表，或 None（无解）
        """
        # 转换为栅格坐标
        start_idx = self._world_to_grid(start)
        goal_idx = self._world_to_grid(goal)

        # 初始化
        start_node = Node(start_idx, g=0, h=self._heuristic(start_idx, goal_idx))

        open_list = [start_node]  # 优先队列
        open_dict = {start_idx: start_node}  # 快速查找
        closed_set = set()

        while open_list:
            # 取出 f 值最小的节点
            current = heapq.heappop(open_list)

            if current.position in closed_set:
                continue

            # 到达终点
            if current.position == goal_idx:
                return self._reconstruct_path(current)

            closed_set.add(current.position)

            # 遍历邻居
            for neighbor_pos in self._get_neighbors(current.position):
                if neighbor_pos in closed_set:
                    continue

                if not self._is_valid(neighbor_pos):
                    continue

                # 计算新的 g 值
                move_cost = self._move_cost(current.position, neighbor_pos)
                new_g = current.g + move_cost

                # 检查是否需要更新
                if neighbor_pos in open_dict:
                    if new_g >= open_dict[neighbor_pos].g:
                        continue

                # 创建/更新邻居节点
                neighbor = Node(
                    neighbor_pos,
                    g=new_g,
                    h=self._heuristic(neighbor_pos, goal_idx),
                    parent=current
                )

                heapq.heappush(open_list, neighbor)
                open_dict[neighbor_pos] = neighbor

        return None  # 无解

    def _heuristic(self, pos, goal):
        """欧几里得距离启发式"""
        return np.sqrt(sum((a-b)**2 for a, b in zip(pos, goal)))

    def _get_neighbors(self, pos):
        """获取 26-邻域"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    neighbors.append((pos[0]+dx, pos[1]+dy, pos[2]+dz))
        return neighbors

    def _is_valid(self, pos):
        """检查位置是否有效（在边界内且无障碍）"""
        x, y, z = pos
        if not (0 <= x < self.grid.shape[0] and
                0 <= y < self.grid.shape[1] and
                0 <= z < self.grid.shape[2]):
            return False
        return self.grid[x, y, z] == 0

    def _move_cost(self, from_pos, to_pos):
        """计算移动代价"""
        return np.sqrt(sum((a-b)**2 for a, b in zip(from_pos, to_pos)))

    def _world_to_grid(self, world_pos):
        """世界坐标转栅格索引"""
        return tuple(int(p / self.resolution) for p in world_pos)

    def _grid_to_world(self, grid_pos):
        """栅格索引转世界坐标"""
        return tuple(p * self.resolution for p in grid_pos)

    def _reconstruct_path(self, node):
        """回溯路径"""
        path = []
        while node:
            path.append(self._grid_to_world(node.position))
            node = node.parent
        return path[::-1]  # 反转，从起点到终点
```

## 七、图示例子

```
假设 2D 简化场景（原理相同）：

地图（S=起点，G=终点，#=障碍）：
  0 1 2 3 4 5
0 S . . . . .
1 . . # # # .
2 . . # G . .
3 . . # . . .
4 . . . . . .

A* 搜索过程：

第1步：展开起点 (0,0)
  邻居：(0,1), (1,0), (1,1)
  计算 f = g + h
  (0,1): f = 1 + 4.47 = 5.47
  (1,0): f = 1 + 4.24 = 5.24 ← 最小，下次展开
  (1,1): f = 1.41 + 3.61 = 5.02 ← 最小，下次展开

... 继续搜索 ...

最终路径：
  0 1 2 3 4 5
0 S → → → ↓ .
1 . . # # # ↓
2 . . # G ← ←
3 . . # . . .
4 . . . . . .
```

## 八、A* 算法特性

```
┌──────────────────────────────────────────────────────────────────────┐
│  优点                                                                │
│  ────                                                                │
│  ✓ 完备性：如果解存在，一定能找到                                    │
│  ✓ 最优性：使用可采纳启发式时，保证找到最短路径                      │
│  ✓ 效率：比 Dijkstra 快（有方向性）                                  │
├──────────────────────────────────────────────────────────────────────┤
│  缺点                                                                │
│  ────                                                                │
│  ✗ 内存占用大：需要存储所有探索过的节点                              │
│  ✗ 高维空间效率低：维度诅咒                                          │
│  ✗ 需要离散化地图：不适合连续空间                                    │
├──────────────────────────────────────────────────────────────────────┤
│  适用场景                                                            │
│  ────────                                                            │
│  • 低维空间（2D/3D）                                                 │
│  • 地图可以离散化为栅格                                              │
│  • 需要保证最优解                                                    │
│  • 实时性要求不是特别高                                              │
└──────────────────────────────────────────────────────────────────────┘
```

## 九、与其他算法对比

```
┌─────────────┬──────────────┬──────────────┬─────────────────────────┐
│   算法      │   最优性     │   完备性     │        时间复杂度       │
├─────────────┼──────────────┼──────────────┼─────────────────────────┤
│   BFS       │      是      │      是      │   O(b^d)                │
├─────────────┼──────────────┼──────────────┼─────────────────────────┤
│  Dijkstra   │      是      │      是      │   O(V + E log V)        │
├─────────────┼──────────────┼──────────────┼─────────────────────────┤
│  贪心搜索   │      否      │      否      │   O(b^m)                │
├─────────────┼──────────────┼──────────────┼─────────────────────────┤
│    A*       │   是(h可采纳)│      是      │   取决于 h 的质量       │
└─────────────┴──────────────┴──────────────┴─────────────────────────┘

b = 分支因子（每个节点的邻居数）
d = 最优解深度
m = 搜索树最大深度
V = 节点数
E = 边数
```

## 十、总结

```
A* 核心要点：

1. f = g + h
   └── g: 已知代价，h: 估计代价

2. 优先探索 f 最小的节点
   └── 兼顾"已走的路"和"剩余的路"

3. 启发式函数要可采纳
   └── h ≤ 实际距离，才能保证最优

4. 适合低维离散空间
   └── 高维空间用 RRT 系列
```
