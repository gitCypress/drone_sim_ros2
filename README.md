# Drone Simulation ROS2 Workspace

本项目是一个基于ROS2的无人机仿真工作空间，集成了AirSim和PX4飞控系统。

## 项目结构

```
src/
├── drone_controller/          # 自开发的无人机控制包（图像保存、键盘遥控、任务执行）
├── px4_msgs/                  # PX4消息定义 (submodule)
└── px4_ros_com/               # PX4与ROS2通信桥接 (submodule)
```

## 依赖说明

### 外部依赖 (Git Submodules)

1. **px4_msgs** (release/1.15)
   - 仓库: https://github.com/PX4/px4_msgs.git
   - 许可证: BSD-3-Clause
   - 用途: PX4飞控系统的ROS2消息定义

2. **px4_ros_com** (release/1.15)
   - 仓库: https://github.com/PX4/px4_ros_com.git
   - 许可证: BSD-3-Clause
   - 用途: PX4与ROS2之间的通信桥接

### 自开发包

- **drone_controller**: 包含图像保存、键盘遥控和任务执行节点

## 克隆项目

### 方式一：完整克隆（推荐）
```bash
git clone --recurse-submodules https://github.com/your-username/drone_sim_ros2.git
```

### 方式二：分步克隆
```bash
git clone https://github.com/your-username/drone_sim_ros2.git
cd drone_sim_ros2
git submodule update --init --recursive
```

**重要提示**: 子模块（px4_msgs和px4_ros_com）是项目运行的必要依赖。如果使用普通的`git clone`命令（不带`--recursive`），子模块目录将为空，需要手动执行子模块初始化命令。

## 构建项目

```bash
cd drone_sim_ros2
colcon build
```

## 运行环境设置

```bash
source install/setup.bash
```

## 许可证

本项目主体代码使用 MIT License，依赖的子模块遵循各自的许可证：
- px4_msgs 和 px4_ros_com 使用 BSD-3-Clause 许可证

## 贡献

欢迎提交Issue和Pull Request！