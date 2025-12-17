# Phase 1: 路径规划

> 目标：实现从起点到终点的无碰撞路径规划

## 🎯 本阶段验收标准

1. 理解 A*、RRT 算法原理
2. 在 3D 栅格地图中规划路径
3. 轨迹平滑处理
4. 无人机按规划路径飞行

## 📚 理论知识

### 1. 路径规划问题定义

```
输入：
  - 起点 (x_start, y_start, z_start)
  - 终点 (x_goal, y_goal, z_goal)
  - 障碍物地图

输出：
  - 无碰撞路径 [(x1,y1,z1), (x2,y2,z2), ...]
```

### 2. A* 算法

**核心思想**：启发式搜索，f(n) = g(n) + h(n)

- g(n): 从起点到当前节点的实际代价
- h(n): 从当前节点到终点的启发式估计（如欧几里得距离）
- f(n): 总代价估计

**伪代码**：
```
1. 将起点加入 open_list
2. while open_list 非空:
     a. 取出 f 值最小的节点 current
     b. if current == goal: 返回路径
     c. 将 current 移入 closed_list
     d. for 每个邻居 neighbor:
          - if neighbor 在 closed_list: 跳过
          - if neighbor 是障碍物: 跳过
          - 计算新的 g 值
          - if neighbor 不在 open_list 或新 g 值更小:
              更新 neighbor 的 g, h, f 值和父节点
              将 neighbor 加入 open_list
3. 返回失败（无路径）
```

### 3. RRT (Rapidly-exploring Random Tree)

**核心思想**：随机采样 + 树扩展

```
1. 初始化树 T，根节点为起点
2. for i = 1 to N:
     a. 随机采样点 x_rand
     b. 找到树中距离 x_rand 最近的节点 x_near
     c. 从 x_near 向 x_rand 方向扩展步长 δ，得到 x_new
     d. if x_near -> x_new 无碰撞:
          将 x_new 加入树
     e. if x_new 接近目标:
          返回路径
3. 返回失败
```

### 4. RRT* 改进

- 在 RRT 基础上增加 rewire 操作
- 选择代价最小的父节点
- 保证渐进最优性

### 5. Minimum Snap 轨迹优化

**目标**：生成平滑、可执行的轨迹

- 最小化 snap (加加速度的导数)
- 满足起终点约束
- 满足中间航点约束

## 💻 代码结构

```
src/phase1_planning/
├── astar_3d.py          # 3D A* 算法
├── rrt_3d.py            # 3D RRT 算法
├── rrt_star_3d.py       # 3D RRT* 算法
├── minimum_snap.py      # 轨迹优化
├── path_planner_node.py # ROS 节点
└── visualizer.py        # 可视化工具
```

## 📝 学习检查清单

- [ ] 理解 A* 的 f = g + h 含义
- [ ] 理解 RRT 的随机采样思想
- [ ] 理解 RRT* 的 rewire 操作
- [ ] 能手动推导简单的 A* 搜索过程
- [ ] 成功运行 3D 路径规划 demo
- [ ] 理解 Minimum Snap 的优化目标
