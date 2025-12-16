# UAV Research Platform

> 从零到科研级的无人机自主飞行系统

## 项目目标

构建一个完整的无人机自主飞行系统，涵盖：
- 路径规划（传统 + AI）
- 视觉/激光雷达感知
- SLAM定位建图
- 轨迹跟踪控制

## 技术栈

| 类别 | 技术 |
|------|------|
| 系统 | Ubuntu 20.04 |
| 中间件 | ROS Noetic |
| 仿真 | Gazebo 11 + PX4 SITL |
| 规划 | A*, RRT*, Minimum Snap |
| 感知 | FAST-LIO2, ORB-SLAM3 |
| 控制 | PID, LQR, MPC |
| AI | PPO/SAC (PyTorch) |

## 项目结构

```
UAV-Research-Platform/
├── docs/                    # 学习文档和笔记
│   ├── phase0_foundation.md
│   ├── phase1_planning.md
│   └── ...
├── scripts/                 # 安装和工具脚本
│   ├── install_ros.sh
│   ├── install_px4.sh
│   └── ...
├── src/                     # 源代码
│   ├── phase0_foundation/   # 基础控制
│   ├── phase1_planning/     # 路径规划
│   ├── phase2_lidar_odom/   # 激光雷达里程计
│   ├── phase3_visual_slam/  # 视觉SLAM
│   ├── phase4_mapping/      # 地图构建
│   ├── phase5_control/      # 控制系统
│   ├── phase6_rl/           # 强化学习
│   └── phase7_integration/  # 系统整合
├── configs/                 # 配置文件
├── launch/                  # ROS launch文件
├── worlds/                  # Gazebo世界文件
└── models/                  # 无人机模型
```

## 学习路线

### Phase 0: 系统地基 (1-2周)
- [x] Ubuntu 20.04 环境
- [ ] ROS Noetic 安装配置
- [ ] PX4 SITL 仿真环境
- [ ] Gazebo 基础操作
- [ ] 验收：无人机起飞→悬停→降落

### Phase 1: 基础路径规划 (3-4周)
- [ ] 2D A* 算法
- [ ] 3D A* 算法
- [ ] RRT / RRT* 算法
- [ ] Minimum Snap 轨迹优化
- [ ] 验收：Gazebo中避障飞行

### Phase 2: 激光雷达里程计 (4-6周)
- [ ] 点云处理基础
- [ ] FAST-LIO2 原理与配置
- [ ] 仿真环境测试
- [ ] 验收：稳定位姿估计

### Phase 3: 视觉SLAM (4-6周)
- [ ] 特征提取与匹配
- [ ] ORB-SLAM2 入门
- [ ] ORB-SLAM3 深入
- [ ] IMU预积分理论
- [ ] 验收：单目/双目/VIO运行

### Phase 4: 地图构建与融合 (2-3周)
- [ ] 点云→Octomap
- [ ] 深度图→Occupancy Grid
- [ ] 多传感器融合
- [ ] 验收：可用于规划的实时地图

### Phase 5: 控制系统 (3-4周)
- [ ] PX4 PID 调参
- [ ] LQR 控制器设计
- [ ] MPC 轨迹跟踪
- [ ] 验收：高速稳定飞行

### Phase 6: AI强化学习 (4-6周)
- [ ] Gym 环境搭建
- [ ] PPO/SAC 算法实现
- [ ] 视觉输入策略
- [ ] 验收：RL规划器成功率>90%

### Phase 7: 系统整合 (持续)
- [ ] 模块解耦与接口设计
- [ ] 对比实验
- [ ] 论文撰写
- [ ] 实机部署准备

## 快速开始

```bash
# 1. 克隆仓库
git clone https://github.com/YOUR_USERNAME/UAV-Research-Platform.git

# 2. 安装依赖 (Ubuntu 20.04)
cd UAV-Research-Platform
./scripts/install_all.sh

# 3. 启动仿真
roslaunch uav_sim base_simulation.launch
```

## 参考资料

- [PX4 官方文档](https://docs.px4.io/)
- [ROS Noetic Wiki](http://wiki.ros.org/noetic)
- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)

## License

MIT License
