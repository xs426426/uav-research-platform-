# 02. RRT/RRT* 采样算法

## 一、为什么需要 RRT

```
A* 的局限性：
┌────────────────────────────────────────────────────────────────────────┐
│  场景：100m × 100m × 50m 的空间，分辨率 0.1m                          │
│                                                                        │
│  栅格数量 = 1000 × 1000 × 500 = 5 亿个格子！                          │
│                                                                        │
│  问题：                                                                │
│  - 内存爆炸：存储这么多格子需要巨大内存                                │
│  - 计算太慢：搜索空间太大                                              │
│  - 维度诅咒：维度越高，问题越严重                                      │
└────────────────────────────────────────────────────────────────────────┘

RRT 的解决思路：
┌────────────────────────────────────────────────────────────────────────┐
│  不把整个空间离散化，而是随机采样！                                    │
│                                                                        │
│  - 只在需要的地方生成节点                                              │
│  - 用连续的线段连接节点                                                │
│  - 自然形成一棵探索树                                                  │
└────────────────────────────────────────────────────────────────────────┘
```

## 二、RRT 算法原理

### 核心思想

```
RRT = Rapidly-exploring Random Tree
     快速探索随机树

核心思想：通过随机采样，逐步扩展一棵树，直到树连接到目标点

形象比喻：
  想象你在一个黑暗的房间里找出口
  - 你随机往某个方向走一步
  - 如果没撞墙，就记住这个位置
  - 重复这个过程，逐渐探索整个房间
  - 最终某一步刚好走到出口附近
```

### 算法流程

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           RRT 算法流程                                   │
└─────────────────────────────────────────────────────────────────────────┘

初始化：树 T 只包含起点

循环 (最多 N 次迭代):
│
├── 1. 随机采样
│       x_rand ← 在空间中随机取一个点
│       (偶尔取目标点，增加找到目标的概率)
│
├── 2. 找最近节点
│       x_nearest ← 树 T 中距离 x_rand 最近的节点
│
├── 3. 扩展
│       x_new ← 从 x_nearest 向 x_rand 方向走一步 (步长 = step_size)
│
│       示意图：
│       x_nearest ●─────────────────────────→ x_rand
│                 │←─ step_size ─→│
│                                 ●
│                               x_new
│
├── 4. 碰撞检测
│       如果 x_nearest → x_new 路径无碰撞:
│           将 x_new 加入树 T
│           设置 x_new 的父节点为 x_nearest
│
└── 5. 检查是否到达目标
        如果 x_new 距离目标 < 阈值:
            连接到目标，返回路径 ✓

如果循环结束仍未找到: 返回失败
```

### 图示过程

```
第1步：起点
        ●S                    G●

第2步：随机采样，扩展
        ●S──●                 G●

第3步：继续随机采样
        ●S──●──●              G●
            │
            ●

第4步：树逐渐生长
        ●S──●──●──●           G●
            │     │
            ●     ●──●

... 继续生长 ...

最终：连接到目标
        ●S──●──●──●──●──●──●──G●
            │     │
            ●     ●──●
                  │
                  ●──●
```

## 三、RRT* 算法改进

### RRT 的问题

```
RRT 找到的路径往往很"丑"：

        ●S──●
            │
            ●──●
               │
               ●──●
                  │
                  ●──●──G●

问题：路径不是最优的，有很多不必要的弯折
```

### RRT* 的改进

```
RRT* = RRT + 两个优化操作

1. 选择最优父节点 (Choose Best Parent)
2. 重新布线 (Rewire)
```

### 改进1：选择最优父节点

```
┌─────────────────────────────────────────────────────────────────────────┐
│  RRT 的做法：x_new 的父节点 = x_nearest（最近的）                       │
│  RRT* 的做法：x_new 的父节点 = 代价最小的（考虑一个邻域内所有节点）     │
└─────────────────────────────────────────────────────────────────────────┘

图示：
                   ┌─────────────────────────────────┐
                   │    邻域半径 r 内的候选父节点    │
                   └─────────────────────────────────┘

                   ●a (cost=5.0)
                        \
         x_nearest ●b (cost=3.0)──→ x_new ●
                        /
                   ●c (cost=4.0)

  RRT:  选 b（最近）  → x_new 的 cost = 3.0 + dist(b, x_new)
  RRT*: 选代价最小的 → 可能选 b，也可能选 a 或 c
        比较: cost_a + dist(a, x_new)
              cost_b + dist(b, x_new)  ← 可能最小
              cost_c + dist(c, x_new)
```

### 改进2：重新布线 (Rewire)

```
┌─────────────────────────────────────────────────────────────────────────┐
│  核心思想：加入新节点后，检查是否能优化周围节点的路径                   │
└─────────────────────────────────────────────────────────────────────────┘

图示：

  重新布线前：                     重新布线后：

  S●───●───●d (cost=6.0)         S●───●───●d
       │                              │
       ●───x_new● (cost=4.0)          ●───x_new●
                                            │
  ●e 的路径：S→...→d→e               ●e 的路径：S→...→x_new→e
  cost_e = 6.0 + 2.0 = 8.0           cost_e = 4.0 + 1.5 = 5.5 ✓ 更优！

检查逻辑：
  对于邻域内的每个节点 x_near:
      if cost(x_new) + dist(x_new, x_near) < cost(x_near):
          将 x_near 的父节点改为 x_new  # 重新布线
```

## 四、RRT* 完整流程

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          RRT* 算法流程                                   │
└─────────────────────────────────────────────────────────────────────────┘

for i in range(N):
│
├── 1. 随机采样
│       x_rand ← Random()
│
├── 2. 找最近节点
│       x_nearest ← Nearest(T, x_rand)
│
├── 3. 扩展
│       x_new ← Steer(x_nearest, x_rand, step_size)
│
├── 4. 碰撞检测
│       if CollisionFree(x_nearest, x_new):
│
│           ┌─────────────────────────────────────────────────────────┐
│           │  ★★★ RRT* 新增部分 ★★★                               │
│           ├─────────────────────────────────────────────────────────┤
│           │  5. 找邻域内所有节点                                    │
│           │      X_near ← Near(T, x_new, radius)                   │
│           │                                                         │
│           │  6. 选择最优父节点                                      │
│           │      x_min ← ChooseBestParent(X_near, x_new)           │
│           │      将 x_new 加入树，父节点为 x_min                    │
│           │                                                         │
│           │  7. 重新布线                                            │
│           │      Rewire(X_near, x_new)                              │
│           └─────────────────────────────────────────────────────────┘
│
└── 8. 检查目标
        if dist(x_new, goal) < threshold:
            找到路径！
```

## 五、代码实现

```python
import numpy as np
import random

class RRTStar3D:
    """3D RRT* 路径规划"""

    def __init__(self, x_range, y_range, z_range,
                 obstacle_checker,
                 step_size=1.0,
                 max_iter=5000,
                 goal_sample_rate=0.1,
                 search_radius=2.0):
        """
        参数:
            x_range, y_range, z_range: 空间范围 (min, max)
            obstacle_checker: 碰撞检测函数
            step_size: 每次扩展的步长
            max_iter: 最大迭代次数
            goal_sample_rate: 采样目标点的概率
            search_radius: RRT* 邻域搜索半径
        """
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        self.obstacle_checker = obstacle_checker
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius

    def plan(self, start, goal):
        """
        规划路径

        参数:
            start: 起点 (x, y, z)
            goal: 终点 (x, y, z)

        返回:
            路径点列表，或 None
        """
        # 初始化树
        self.start = np.array(start)
        self.goal = np.array(goal)

        # 节点列表：每个节点 = [position, parent_index, cost]
        self.nodes = [[self.start, -1, 0.0]]

        for i in range(self.max_iter):
            # 1. 随机采样
            x_rand = self._sample()

            # 2. 找最近节点
            nearest_idx = self._nearest(x_rand)
            x_nearest = self.nodes[nearest_idx][0]

            # 3. 扩展
            x_new = self._steer(x_nearest, x_rand)

            # 4. 碰撞检测
            if self._collision_free(x_nearest, x_new):

                # 5. 找邻域节点 (RRT* 特有)
                near_indices = self._near(x_new)

                # 6. 选择最优父节点 (RRT* 特有)
                best_parent_idx, min_cost = self._choose_best_parent(
                    near_indices, x_new
                )

                # 添加新节点
                new_idx = len(self.nodes)
                self.nodes.append([x_new, best_parent_idx, min_cost])

                # 7. 重新布线 (RRT* 特有)
                self._rewire(near_indices, new_idx, x_new)

                # 8. 检查是否到达目标
                if np.linalg.norm(x_new - self.goal) < self.step_size:
                    return self._extract_path(new_idx)

        return None  # 未找到路径

    def _sample(self):
        """随机采样"""
        if random.random() < self.goal_sample_rate:
            return self.goal.copy()
        else:
            return np.array([
                random.uniform(*self.x_range),
                random.uniform(*self.y_range),
                random.uniform(*self.z_range)
            ])

    def _nearest(self, x_rand):
        """找最近节点"""
        distances = [np.linalg.norm(node[0] - x_rand) for node in self.nodes]
        return int(np.argmin(distances))

    def _steer(self, x_from, x_to):
        """从 x_from 向 x_to 方向走一步"""
        direction = x_to - x_from
        distance = np.linalg.norm(direction)

        if distance < self.step_size:
            return x_to.copy()
        else:
            return x_from + (direction / distance) * self.step_size

    def _collision_free(self, x_from, x_to):
        """检查路径是否无碰撞"""
        # 沿路径采样检测
        distance = np.linalg.norm(x_to - x_from)
        n_checks = max(int(distance / 0.1), 2)

        for i in range(n_checks + 1):
            t = i / n_checks
            point = x_from + t * (x_to - x_from)
            if self.obstacle_checker(point):
                return False
        return True

    def _near(self, x_new):
        """找邻域内的所有节点"""
        indices = []
        for i, node in enumerate(self.nodes):
            if np.linalg.norm(node[0] - x_new) < self.search_radius:
                indices.append(i)
        return indices

    def _choose_best_parent(self, near_indices, x_new):
        """选择代价最小的父节点"""
        best_parent = None
        min_cost = float('inf')

        for idx in near_indices:
            node = self.nodes[idx]
            x_near = node[0]
            cost = node[2]  # 到达 x_near 的代价

            # 计算经过 x_near 到达 x_new 的总代价
            new_cost = cost + np.linalg.norm(x_new - x_near)

            if new_cost < min_cost and self._collision_free(x_near, x_new):
                min_cost = new_cost
                best_parent = idx

        return best_parent, min_cost

    def _rewire(self, near_indices, new_idx, x_new):
        """重新布线：检查是否能优化邻域节点的路径"""
        new_cost = self.nodes[new_idx][2]

        for idx in near_indices:
            if idx == self.nodes[new_idx][1]:  # 跳过父节点
                continue

            x_near = self.nodes[idx][0]
            old_cost = self.nodes[idx][2]

            # 计算经过 x_new 到达 x_near 的新代价
            potential_cost = new_cost + np.linalg.norm(x_near - x_new)

            # 如果新路径更优且无碰撞
            if potential_cost < old_cost and self._collision_free(x_new, x_near):
                self.nodes[idx][1] = new_idx  # 更新父节点
                self.nodes[idx][2] = potential_cost  # 更新代价

    def _extract_path(self, goal_idx):
        """提取路径"""
        path = [self.goal]
        idx = goal_idx

        while idx != -1:
            path.append(self.nodes[idx][0])
            idx = self.nodes[idx][1]

        return path[::-1]  # 反转
```

## 六、RRT vs RRT* 对比

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       RRT vs RRT* 对比                                  │
├──────────────┬────────────────────┬─────────────────────────────────────┤
│     特性     │       RRT          │           RRT*                      │
├──────────────┼────────────────────┼─────────────────────────────────────┤
│   最优性     │  概率完备但非最优  │  渐进最优（迭代越多越接近最优）     │
├──────────────┼────────────────────┼─────────────────────────────────────┤
│   计算量     │      O(n)          │  O(n log n) - 需要邻域搜索          │
├──────────────┼────────────────────┼─────────────────────────────────────┤
│  路径质量    │    可能很差        │         较好                        │
├──────────────┼────────────────────┼─────────────────────────────────────┤
│   核心操作   │    仅扩展          │  扩展 + 选最优父节点 + 重新布线     │
├──────────────┼────────────────────┼─────────────────────────────────────┤
│   适用场景   │  快速找可行解      │  需要较优路径                       │
└──────────────┴────────────────────┴─────────────────────────────────────┘
```

## 七、路径效果对比

```
RRT 的路径（可能很丑）：

  S●──●
      │
      ●──●
         │
         ●──●
            │
            ●──●
               │
               ●G

RRT* 的路径（经过优化）：

  S●──●──●──●──●G

  经过重新布线，路径变得更直接
```

## 八、参数调优

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        关键参数调优指南                                  │
├────────────────┬────────────────────────────────────────────────────────┤
│     参数       │                    调优建议                            │
├────────────────┼────────────────────────────────────────────────────────┤
│   step_size    │ 太大：可能跳过狭窄通道                                 │
│                │ 太小：收敛慢                                           │
│                │ 建议：空间尺寸的 2-5%                                  │
├────────────────┼────────────────────────────────────────────────────────┤
│   max_iter     │ 复杂环境需要更多迭代                                   │
│                │ 简单环境 1000-5000 就够                                │
├────────────────┼────────────────────────────────────────────────────────┤
│goal_sample_rate│ 太大：可能陷入局部                                     │
│                │ 太小：找到目标慢                                       │
│                │ 建议：5-20%                                            │
├────────────────┼────────────────────────────────────────────────────────┤
│ search_radius  │ RRT* 专用，影响优化质量                                │
│  (RRT*)        │ 建议：2-5 倍 step_size                                 │
└────────────────┴────────────────────────────────────────────────────────┘
```

## 九、总结

```
RRT 系列算法要点：

1. 随机采样 + 逐步扩展
   └── 不需要离散化整个空间

2. 概率完备
   └── 只要解存在，给足够时间一定能找到

3. RRT* 的两个优化
   ├── 选择最优父节点
   └── 重新布线

4. 适用场景
   ├── 高维空间
   ├── 复杂环境
   └── 连续空间规划
```
