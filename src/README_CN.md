# B2Z1 ROS2 工作空间

本工作空间包含一个用于宇树（Unitree）B2 + Z1 的轻量级 ROS2 控制骨架，以及一条 MuJoCo 验证路径，用于在真机测试前验证命令流、姿态和演示行为。

当前仓库有意将以下两个关注点分离：

- **MuJoCo 验证**：针对仅 B2 和仅 Z1 的演示进行快速迭代。
- **真机集成**：尽可能复用官方的宇树代码路径。

## 1. 仓库范围

本工作空间中的主要功能包包括：

- `b2z1_msgs`：自定义 ROS2 消息定义。
- `b2z1_b2_bridge`：B2 命令桥接器。
- `b2z1_coordinator`：可选的协调节点，用于未来统一的命令路由。
- [b2z1_examples](file:///home/liu/b2z1_ros2_ws/src/b2z1_examples/resource/b2z1_examples)：用于仅 B2 和仅 Z1 验证的演示脚本。
- `b2z1_mujoco_bridge`：将 ROS2 演示命令桥接到 MuJoCo 底层命令的桥梁。
- `b2z1_bringup`：用于 MuJoCo 验证演示的启动文件。
- `simulate/b2z1_mujoco`：B2Z1 MuJoCo 场景和资源文件。
- `unitree_api`：从官方宇树 ROS2 工作空间复制并保留在此处的供应商依赖项，用于可重现的本地构建。

## 2. 从参考代码中复用的内容

另请参阅 [THIRD_PARTY_NOTICES.md](./THIRD_PARTY_NOTICES.md) 以获取供应商依赖项和归属说明。

本项目遵循以下原则：只要可行，就复用官方/参考实现，仅在组合后的 B2+Z1 MuJoCo 路径需要时添加自定义逻辑。

### 2.1 直接从 `unitree_ros2` 复制或强力适配的内容

参考根目录：
- `/home/liu/dwbc_ws/unitree_ws/src/unitree_ros2`

主要复用部分：
- [b2z1_b2_bridge/include/b2z1_b2_bridge/ros2_b2_sport_client.hpp](file:///home/liu/b2z1_ros2_ws/src/b2z1_b2_bridge/include/b2z1_b2_bridge/ros2_b2_sport_client.hpp)
- [b2z1_b2_bridge/src/ros2_b2_sport_client.cpp](file:///home/liu/b2z1_ros2_ws/src/b2z1_b2_bridge/src/ros2_b2_sport_client.cpp)

这些文件基于来自以下位置的官方 B2 运动客户端工具：
- `unitree_ros2/example/src/include/common/ros2_b2_sport_client.h`
- `unitree_ros2/example/src/src/common/ros2_b2_sport_client.cpp`

B2 模式语义也遵循官方示例：
- `balance_stand`（平衡站立）
- `stand_down`（趴下/解除站立）
- `damp`（阻尼模式）
- `move`（移动）

MuJoCo 桥接中的 B2 站立时序和阶段结构对齐自：
- `unitree_ros2/example/src/src/b2/b2_stand_example.cpp`

### 2.2 直接从 `z1_ros2` 复制或强力适配的内容

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

### 2.3 直接复制的外部模型资源

位于以下目录的 MuJoCo 模型资源：
- `simulate/b2z1_mujoco`

这些资源是从早期的 B2Z1 模型工作中复制而来，随后经过调整以适用于宇树 MuJoCo 回路验证。特别是：
- 基础 B2Z1 模型/资源是从早期的训练/遗留模型源复制的，
- 然后进行调整以满足宇树 MuJoCo 控制预期，
- 并在需要时进一步对齐 Go2 场景模板约定。

### 2.4 保留在此仓库中的供应商依赖项

以下功能包被有意复制到此工作空间中，以便工作空间可以可重现地构建，而无需要求每个用户手动重建相同的子目录布局：
- `src/unitree_api`

来源谱系：
- 复制自 `unitree_ros2/cyclonedds_ws/src/unitree/unitree_api`
- 上游包元数据声明维护者为 `Unitree <unitree@unitree.com>`，许可证为 `BSD 3-Clause License`

本仓库不声称拥有 `unitree_api` 的著作权；它作为供应商依赖项保留在此处。

### 2.5 本工作空间中新增的自定义代码

以下部分为本工作空间的自定义内容：
- `b2z1_msgs`
- `b2z1_coordinator`
- [b2z1_examples](file:///home/liu/b2z1_ros2_ws/src/b2z1_examples/resource/b2z1_examples)
- `b2z1_mujoco_bridge`
- `b2z1_bringup`

这些不是官方上游文件。添加它们是为了：
- 将工作空间组织为 ROS2 功能包，
- 连接演示脚本到 MuJoCo 回路控制，
- 为后续的高级策略/控制工作提供基础。

## 3. 重要边界：MuJoCo 路径与真机路径

### MuJoCo 验证路径

当前的 MuJoCo 演示大致执行流程如下：

- [b2z1_examples](file:///home/liu/b2z1_ros2_ws/src/b2z1_examples/resource/b2z1_examples)
- `b2z1_mujoco_bridge`
- `rt/lowcmd`
- `unitree_mujoco`

对于 Z1，这意味着 MuJoCo 目前使用直接的底层 PD 风格命令执行。

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



## 4. 仅 MuJoCo 工作空间的依赖项

要在另一台计算机上运行当前的 MuJoCo 演示，您至少需要：

来自本工作空间：
- `b2z1_msgs`
- [b2z1_examples](file:///home/liu/b2z1_ros2_ws/src/b2z1_examples/resource/b2z1_examples)
- `b2z1_mujoco_bridge`
- `b2z1_bringup`
- `simulate/b2z1_mujoco`

外部依赖项：
- 单独安装的 `unitree_rl_mjlab` 提供 `unitree_mujoco`

真机工作的可选依赖项：
- `b2z1_b2_bridge`
- `b2z1_coordinator`
- `unitree_api`
- 官方 `z1_ros2` 包

## 5. 构建说明

### 5.1 通用构建

```bash
cd ~/b2z1_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash


如果 conda 处于激活状态，建议先退出：


```bash
conda deactivate
```

## 6. MuJoCo 使用方法

### 6.1 启动 Unitree MuJoCo

假设 unitree_rl_mjlab 已安装在机器的其他位置：

```bash
cd /path/to/unitree_rl_mjlab/simulate/build
./unitree_mujoco --network lo --robot b2 --scene /home/liu/b2z1_ros2_ws/src/simulate/b2z1_mujoco/xmls/b2z1_ctrl_stage1.xml
```

Note:
- 在 unitree_rl_mjlab/simulate/config.yaml 中，除非实际可用手柄，否则建议设置 use_joystick: 0。

### 6.2 运行 B2 站立演示
另开终端：
```bash
cd ~/b2z1_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch b2z1_bringup b2_only_stand.launch.py network_interface:=lo
```

### 6.3 运行 B2 stand-then-down demo（两个差不多）

```bash
ros2 launch b2z1_bringup b2_only_stand_then_down.launch.py network_interface:=lo
```

### 6.4 运行 Z1-only reach demo

```bash
ros2 launch b2z1_bringup z1_only_reach.launch.py network_interface:=lo
```

## 7. 真机使用方法

### 7.1 B2 真机

对于真实的 B2 使用，请优先使用官方宇树路径和语义：
- official `unitree_ros2` examples,
- `unitree_api`,
- sport client semantics.

本工作空间包含：
- `b2z1_b2_bridge`

其意图是作为一个薄桥接层，而非替代官方 B2 通信语义。

### 7.2 Z1 真机

对于真实的 Z1 使用，请优先使用官方 z1_ros2 堆栈：:
- `z1_bringup`
- `z1_hardware_interface`
- `joint_trajectory_controller`
- `FollowJointTrajectory`

The current MuJoCo demos should be treated as:
- 结构的验证，
- 关键点选择的验证，
- 高级控制组织的验证，

而不是真机行为的最终证明。


## 8. 已知的 MuJoCo 特定适配

代码试图最小化自定义行为，但为了稳定性仍保留了一些仅针对 MuJoCo 的适配：
- `ground_support` for B2 during Z1-only validation,
- a `gentle retract` stage in the Z1-only demo,
- small protective adaptations in the B2 MuJoCo stand path to avoid instability in the combined B2+Z1 model.

这些是有意为之的仿真适配，不应与最终的真机执行链混淆。

