# B2Z1 ROS2 工作空间

本工作空间包含一个用于宇树（Unitree）B2 + Z1 的轻量级 ROS2 控制骨架，以及一条 MuJoCo 验证路径，用于在真机测试前验证命令流、姿态和演示行为。

当前仓库明确分成两条主线：

- **真机主线**：B2 与 Z1 的真实执行路径，尽量贴近官方代码
- **仿真主线**：MuJoCo 验证路径，用于观察控制效果是否符合预期

## 1. 两条主线

### 1.1 真机主线

目标：

- 让 B2 的真机 lowcmd 路径清楚可用
- 让 Z1 的真机轨迹控制路径清楚可用

真机主线涉及的主要包：

- `real/b2/b2z1_b2_lowcmd`
- `real/z1/b2z1_z1_control`
- `shared/b2/b2z1_b2_trajectories`
- `shared/z1/b2z1_z1_trajectories`
- `bringup/b2z1_bringup`
- `third_party/unitree/unitree_go`
- `third_party/unitree/unitree_api`

真机 launch 入口：

- `ros2 launch b2z1_bringup real/b2_real_lowcmd_stand.launch.py`
- `ros2 launch b2z1_bringup real/z1_real_waypoint.launch.py`

### 1.2 MuJoCo 仿真主线

目标：

- 在真机测试前先验证命令流
- 在 MuJoCo 里验证姿态、时序和验证行为

仿真主线涉及的主要包：

- `sim/mujoco/b2z1_mujoco_bridge`
- `sim/mujoco/b2z1_examples`
- `sim/mujoco/simulate/b2z1_mujoco`
- `shared/common/b2z1_msgs`
- `shared/b2/b2z1_b2_trajectories`
- `shared/z1/b2z1_z1_trajectories`
- `bringup/b2z1_bringup`

仿真 launch 入口：

- `ros2 launch b2z1_bringup b2_stand_validation_sim.launch.py network_interface:=lo`
- `ros2 launch b2z1_bringup z1_waypoint_validation_sim.launch.py network_interface:=lo`
- 内部会继续落到 `launch/mujoco/*`

### 1.3 重要边界

当前仓库 **并不包含** 完整的 `B2Z1` 联合控制主线。

当前主线更准确地说是：

- B2 真机控制
- Z1 真机控制
- 对这些控制语义的 MuJoCo 验证

## 2. 仓库范围

当前工作空间目录树：

```text
src/
├── bringup/
│   └── b2z1_bringup
├── real/
│   ├── b2/
│   │   └── b2z1_b2_lowcmd
│   └── z1/
│       └── b2z1_z1_control
├── shared/
│   ├── b2/
│   │   └── b2z1_b2_trajectories
│   ├── common/
│   │   └── b2z1_msgs
│   └── z1/
│       └── b2z1_z1_trajectories
├── sim/
│   └── mujoco/
│       ├── b2z1_examples
│       ├── b2z1_mujoco_bridge
│       └── simulate/b2z1_mujoco
├── third_party/
│   └── unitree/
│       ├── unitree_api
│       └── unitree_go
└── legacy/
    └── b2z1_coordinator
```

当前分层原则：

- `bringup/`：只放启动入口
- `real/`：真机执行后端
- `shared/`：共享轨迹、共享消息、共享运动语义
- `sim/`：MuJoCo 仿真执行和验证入口
- `third_party/`：保留在仓库内的上游依赖
- `legacy/`：历史上的联合控制实验代码，不属于当前主线

本工作空间中的主要功能包包括：

- `b2z1_b2_lowcmd`：B2 真机 lowcmd 执行后端。
- `b2z1_z1_control`：Z1 真机轨迹控制执行后端，对齐官方 `z1_ros2`。
- `b2z1_b2_trajectories`：B2 共享轨迹层。
- `b2z1_z1_trajectories`：Z1 共享轨迹层。
- `b2z1_msgs`：仿真验证路径使用的实验性命令消息定义。
- `b2z1_examples`：仿真验证脚本。
- `b2z1_mujoco_bridge`：MuJoCo 仿真执行后端。
- `b2z1_bringup`：实机/仿真启动入口整理包。
- `simulate/b2z1_mujoco`：B2Z1 MuJoCo 场景和资源文件。

当前主包之间的关系：

- `b2z1_b2_lowcmd`：依赖 `b2z1_b2_trajectories`、`unitree_go`、`unitree_api`
- `b2z1_z1_control`：依赖 `b2z1_z1_trajectories` 和官方 `z1_bringup`
- `b2z1_mujoco_bridge`：依赖 `b2z1_msgs`、`b2z1_b2_lowcmd`、`b2z1_b2_trajectories`
- `b2z1_examples`：向 `b2z1_mujoco_bridge` 发布实验性 MuJoCo 验证命令
- `b2z1_bringup`：统一提供真机与仿真的 launch 入口

## 3. 从参考代码中复用的内容

另请参阅 [THIRD_PARTY_NOTICES.md](./THIRD_PARTY_NOTICES.md) 以获取归属说明。

本项目遵循以下原则：只要可行，就复用官方/参考实现，仅在组合后的 B2+Z1 MuJoCo 路径需要时添加自定义逻辑。

### 3.1 与 `unitree_ros2` 强对齐的内容

参考根目录：
- `/home/liu/dwbc_ws/unitree_ws/src/unitree_ros2`

MuJoCo 桥接中的 B2 lowcmd 起立时序和阶段结构主要对齐自：
- `unitree_ros2/example/src/src/b2/b2_stand_example.cpp`

这种对齐主要体现在：
- B2 多阶段起立时序，
- 起立目标姿态组织，
- 面向 lowcmd 的 B2 控制方向。

### 3.2 直接从 `z1_ros2` 复制或强力适配的内容

参考根目录：
- `/home/liu/dwbc_ws/unitree_ws/src/z1_ros2`

主要复用思路：
- Z1 关节命名和顺序保持与 `z1_ros2` 一致。
- 演示中使用的 Z1 姿态关键点尽可能与 `z1_ros2/z1_examples/z1_examples/waypoint_test.py` 对齐：
  - `home`（ home 位）
  - `reach`（伸展位）
  - `wrist pose`（手腕姿态）

对于真机使用，预期的官方路径仍然是：
- `z1_bringup`
- `z1_hardware_interface`
- `joint_trajectory_controller`
- `FollowJointTrajectory`

### 3.3 直接复制的外部模型资源

位于以下目录的 MuJoCo 模型资源：
- `simulate/b2z1_mujoco`

这些资源是从早期的 B2Z1 模型工作中复制而来，随后经过调整以适用于宇树 MuJoCo 回路验证。特别是：
- 基础 B2Z1 模型/资源是从早期的训练/遗留模型源复制的，
- 然后进行调整以满足宇树 MuJoCo 控制预期，
- 并在需要时进一步对齐 Go2 场景模板约定。

### 3.4 本工作空间中新增的自定义代码

以下部分为本工作空间的自定义内容：
- `b2z1_msgs`
- `b2z1_b2_lowcmd`
- `b2z1_z1_control`
- `b2z1_b2_trajectories`
- `b2z1_z1_trajectories`
- `b2z1_examples`
- `b2z1_mujoco_bridge`
- `b2z1_bringup`

这些不是官方上游文件。添加它们是为了：
- 将工作空间组织为 ROS2 功能包，
- 连接演示脚本到 MuJoCo 回路控制，
- 为后续的高级策略/控制工作提供基础。

## 4. 真机路径与仿真路径的关系

### MuJoCo 验证路径

当前的 MuJoCo 验证大致执行流程如下：

- `b2z1_examples`
- `b2z1_mujoco_bridge`
- `rt/lowcmd`
- `unitree_mujoco`

当前整理后的职责划分是：
- `b2z1_b2_lowcmd`：B2 实机后端
- `b2z1_z1_control`：Z1 实机后端
- `b2z1_b2_trajectories`：B2 共享轨迹
- `b2z1_z1_trajectories`：Z1 实机/仿真共享轨迹来源
- `b2z1_mujoco_bridge`：MuJoCo 后端
- `b2z1_examples`：仿真验证节点
- `b2z1_bringup`：启动入口
- `b2z1_msgs`：仿真消息

对于 Z1，这意味着真机与仿真现在已经共享同一个官方 waypoint 来源，但 MuJoCo 仍然使用直接的底层 PD 风格命令执行。
对于 B2，则意味着当前主线明确围绕 lowcmd 风格控制组织，而不是高层 sport client。

### 官方真机 Z1 路径

`z1_ros2` 中经过官方验证的 Z1 堆栈通过以下路径执行：

- 轨迹/动作命令
- `joint_trajectory_controller`
- 硬件接口
- 真机

这就是为什么 MuJoCo 演示路径与 `z1_ros2` 执行路径**不完全相同**的原因，即使关键姿态点是对齐的。

### 这意味着

当前的 MuJoCo 验证**确实证明了**：
- 工作空间/包结构是合理的，
- 命令定义是合理的，
- 演示逻辑和姿态组织是可用的，
- 代码库是后续策略控制工作的实用基础。

当前的 MuJoCo 验证**并未完全证明**：
- 在真机上的相同行为，
- 与 `z1_ros2` 轨迹控制器执行的完全等价性，
- 最终的真机安全性。



## 5. 仅 MuJoCo 工作空间的依赖项

要在另一台计算机上运行当前的 MuJoCo 演示，您至少需要：

来自本工作空间：
- `b2z1_b2_trajectories`
- `b2z1_z1_trajectories`
- `b2z1_msgs`
- `b2z1_examples`
- `b2z1_mujoco_bridge`
- `b2z1_bringup`
- `simulate/b2z1_mujoco`

外部依赖项：
- 单独安装的 `unitree_rl_mjlab` 提供 `unitree_mujoco`

真机工作的外部依赖项：
- 官方 `z1_ros2` 包

## 6. 构建说明

### 6.1 通用构建

```bash
cd ~/b2z1_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

如果 conda 处于激活状态，建议先退出：

```bash
conda deactivate
```

## 7. MuJoCo 使用方法

### 7.1 启动 Unitree MuJoCo

假设 unitree_rl_mjlab 已安装在机器的其他位置：

```bash
cd /path/to/unitree_rl_mjlab/simulate/build
./unitree_mujoco --network lo --robot b2 --scene /home/liu/b2z1_ros2_ws/src/sim/mujoco/simulate/b2z1_mujoco/xmls/b2z1_ctrl_stage1.xml
```

Note:
- 在 unitree_rl_mjlab/simulate/config.yaml 中，除非实际可用手柄，否则建议设置 use_joystick: 0。

### 7.2 运行 MuJoCo 验证入口

```bash
cd ~/b2z1_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch b2z1_bringup b2_stand_validation_sim.launch.py network_interface:=lo
```

```bash
ros2 launch b2z1_bringup z1_waypoint_validation_sim.launch.py network_interface:=lo
```

验证目标：

- `b2_stand_validation_sim.launch.py`
  - 验证 B2 真机 lowcmd 站立路径的控制语义
- `z1_waypoint_validation_sim.launch.py`
  - 验证 Z1 真机 waypoint 路径的姿态与时序语义

## 8. 真机方向

### 8.1 B2 真机 lowcmd 站立测试

直接真机 lowcmd launch 为：

```bash
ros2 launch b2z1_bringup real/b2_real_lowcmd_stand.launch.py
```

### 8.2 B2 真机定位

本工作空间对 B2 的目标方向是基于 lowcmd 的真机控制，而不是高层 sport client 路径。

也就是说，本仓库更适合作为以下工作的基础：
- 面向关节角的 B2 控制，
- 后续策略直接输出关节空间目标，
- 尽量缩小仿真执行层和未来低层真机控制层之间的风格差异。

### 8.3 Z1 真机

对于真实的 Z1 使用，请优先使用官方 `z1_ros2` 堆栈：
- `z1_bringup`
- `z1_hardware_interface`
- `joint_trajectory_controller`
- `FollowJointTrajectory`

当前 MuJoCo 演示更适合被看作：
- 结构验证，
- 关键点选择验证，
- lowcmd 风格仿真验证，

而不是真机行为的最终证明。


## 9. 已知的 MuJoCo 特定适配

代码试图最小化自定义行为，但为了稳定性仍保留了一些仅针对 MuJoCo 的适配：
- `ground_support`：用于 Z1-only 验证时给 B2 提供轻支撑，
- `gentle retract`：用于 Z1-only 演示中的回收过渡，
- B2 MuJoCo 起立路径中少量保护性适配，用于避免组合 B2+Z1 模型中的不稳定。

这些是有意为之的仿真适配，不应与最终的真机执行链混淆。
