# B2Z1 ROS 2 控制工作空间

本项目为 Unitree B2 四足机器人和 Z1 机械臂提供一套轻量级 ROS 2 控制骨架。当前重点是先分别打通 B2、Z1 的真机执行链路，再通过 MuJoCo 验证姿态、时序和联合命令，最终演进为统一的 B2+Z1 策略控制框架。

当前状态：

- B2 真机：已实现基于 `LowCmd` 的固定站立示例。
- Z1 真机：已接入官方 `joint_trajectory_controller` 和 `FollowJointTrajectory` waypoint 示例。
- B2 仿真：已实现 MuJoCo 站立验证。
- Z1 仿真：已实现 MuJoCo waypoint 验证。
- B2+Z1 联合仿真：已实现同一时间线下的联合动作验证。
- B2+Z1 联合真机：尚未实现统一协调器和安全状态机。

> 安全提示：仓库最新整理版本与“疑似 Z1 保险丝烧坏”实机问题相关。在故障原因、供电、负载、关节限位和控制参数确认前，不要直接运行 Z1 或 B2+Z1 真机动作。

## 1. 工作空间结构

```text
b2z1_ros2_ws/
├── .gitignore
├── b2z1_dependencies.repos        # 固定外部源码依赖版本
├── src/
│   ├── b2z1_ctr/                  # 本项目代码
│   │   ├── bringup/
│   │   │   └── b2z1_bringup/     # 真机与仿真 launch 入口
│   │   ├── real/
│   │   │   ├── b2/
│   │   │   │   └── b2z1_b2_lowcmd/
│   │   │   └── z1/
│   │   │       └── b2z1_z1_control/
│   │   ├── shared/
│   │   │   ├── b2/
│   │   │   │   └── b2z1_b2_trajectories/
│   │   │   ├── common/
│   │   │   │   └── b2z1_msgs/
│   │   │   └── z1/
│   │   │       └── b2z1_z1_trajectories/
│   │   └── sim/mujoco/
│   │       ├── b2z1_examples/
│   │       ├── b2z1_mujoco_bridge/
│   │       └── simulate/b2z1_mujoco/
│   ├── unitree_ros2/              # 上游依赖，不纳入本仓库 Git
│   ├── z1_ros2/                   # 上游依赖，不纳入本仓库 Git
│   ├── tools/                     # 数据分析工具
│   └── docs/                      # 本地交接文档
├── build/
├── install/
└── log/
```

分层约定：

- `bringup/`：只负责启动和组合节点。
- `real/`：B2、Z1 真机执行后端。
- `shared/`：共享消息、轨迹参数和动作语义。
- `sim/`：MuJoCo bridge、验证节点和 B2Z1 模型。
- `unitree_ros2/`、`z1_ros2/`：由工作空间根目录的 `.repos` 文件固定版本。

## 2. 功能包

### 2.1 本项目功能包

| 包 | 类型 | 功能 |
|---|---|---|
| `b2z1_bringup` | CMake | 汇总 B2、Z1、联合仿真和真机 launch |
| `b2z1_b2_lowcmd` | C++ | B2 真机 `LowCmd` 站立控制、CRC 和 motion switch |
| `b2z1_b2_trajectories` | C++ header/config | B2 分阶段站立轨迹、下蹲姿态和仿真 PD 参数 |
| `b2z1_z1_control` | Python | Z1 真机 `FollowJointTrajectory` 客户端 |
| `b2z1_z1_trajectories` | Python | Z1 官方 waypoint 和 MuJoCo 安全 waypoint |
| `b2z1_msgs` | ROS interface | MuJoCo 验证命令 `MujocoDemoCommand` |
| `b2z1_examples` | Python | B2、Z1 和 B2+Z1 联合仿真验证节点 |
| `b2z1_mujoco_bridge` | C++ | 将验证命令转换成 Unitree DDS `rt/lowcmd` |

### 2.2 上游功能包

当前工作空间共可识别 17 个 ROS 2 包。除上述 8 个项目包外，还包括：

- Unitree：`unitree_api`、`unitree_go`、`unitree_hg`、`unitree_ros2_example`。
- Z1：`z1_bringup`、`z1_description`、`z1_examples`、`z1_hardware_interface`、`z1_moveit`。

外部源码版本记录在工作空间根目录的 `b2z1_dependencies.repos` 中。

## 3. 控制链路

### 3.1 B2 真机

```text
b2_real_lowcmd_stand.launch.py
  -> b2_real_lowcmd_stand_node
  -> 等待 /lowstate
  -> CheckMode / ReleaseMode
  -> B2StandTrajectory
  -> LowCmd + CRC
  -> /lowcmd
  -> B2
```

节点先等待有效状态并释放官方运动控制服务，然后启动 2 ms 控制周期。它读取当前 12 个关节位置作为起点，再执行配置文件中的多阶段站立轨迹。

主要文件：

- `real/b2/b2z1_b2_lowcmd/src/b2_real_lowcmd_stand_node.cpp`
- `real/b2/b2z1_b2_lowcmd/include/b2z1_b2_lowcmd/b2/b2_motion_switch_client.hpp`
- `shared/b2/b2z1_b2_trajectories/include/b2z1_b2_trajectories/b2_stand_trajectory.hpp`
- `shared/b2/b2z1_b2_trajectories/config/b2_stand_trajectory.yaml`

注意：B2 真机节点默认使用 `kp=1000`、`kd=10`，MuJoCo bridge 使用 YAML 中的分关节增益，二者不能视为等价参数。

### 3.2 Z1 真机

```text
z1_real_waypoint.launch.py
  -> 官方 z1_bringup/z1.launch.py
  -> z1_hardware_interface
  -> joint_trajectory_controller
  -> z1_waypoint_test
  -> FollowJointTrajectory
  -> Z1
```

Z1 六关节顺序为：

```text
joint1, joint2, joint3, joint4, joint5, joint6
```

真机 launch 启动官方硬件接口和控制器后，会等待固定的 6 秒并自动运行 waypoint 节点。目前没有人工确认、故障联锁或目标偏差检查，因此排除保险丝问题之前不要运行。

主要文件：

- `real/z1/b2z1_z1_control/b2z1_z1_control/z1_waypoint_test.py`
- `shared/z1/b2z1_z1_trajectories/b2z1_z1_trajectories/sequences.py`
- `../z1_ros2/z1_bringup/launch/z1.launch.py`
- `../z1_ros2/z1_bringup/config/z1_controllers.yaml`

### 3.3 MuJoCo bridge

```text
b2z1_examples
  -> /mujoco_demo_command
  -> b2z1_mujoco_bridge
  -> rt/lowcmd（2 ms）
  -> unitree_mujoco
```

`MujocoDemoCommand` 当前包含：

```text
string b2_mode
float32 vx
float32 vy
float32 vyaw
float32[6] z1_q
float32 duration
```

bridge 中的电机映射为：

- B2：索引 `0..11`。
- Z1：索引 `12..17`。
- 保留电机：索引 `18..19`，保持 stop。

支持的 B2 模式：

- `damp`：停止 B2 电机输出。
- `balance_stand`：执行 B2 分阶段站立轨迹。
- `stand_down`：插值到下蹲姿态。
- `ground_support`：保持下蹲，并使用缩放后的 PD 增益支撑 Z1 仿真。
- `move`：当前只按站立保持处理，`vx/vy/vyaw` 尚未转成行走控制。

### 3.4 B2+Z1 联合仿真

联合验证节点在同一消息和同一时间线上同时给出 B2 模式与 Z1 六关节目标：

```text
1.0 s   B2 开始站立       + Z1 移动到保守伸展姿态
10.0 s  B2 保持站立       + Z1 移动到第二个 waypoint
13.0 s  B2 开始下蹲       + Z1 返回零位
```

相关文件：

- `sim/mujoco/b2z1_examples/b2z1_examples/b2z1_joint_validation.py`
- `bringup/b2z1_bringup/launch/b2z1_joint_validation_sim.launch.py`
- `bringup/b2z1_bringup/launch/mujoco/b2z1_joint_validation.launch.py`

这条链路证明 bridge 可以在一个控制周期内同时更新 B2 和 Z1，但不代表真机联合控制或真机安全性已经验证。

## 4. 依赖和构建

### 4.1 获取源码依赖

新工作空间中执行：

```bash
cd /home/liu/b2z1_ros2_ws
vcs import src < b2z1_dependencies.repos
```

还需要单独准备提供 `unitree_mujoco` 的 `unitree_rl_mjlab`。该项目没有包含在当前 `.repos` 文件中。

### 4.2 构建

如果启用了 Conda，先退出，避免 ROS 2 Humble 的 Python 3.10 与 Conda Python 冲突：

```bash
conda deactivate
```

然后构建：

```bash
cd /home/liu/b2z1_ros2_ws
source /opt/ros/humble/setup.bash

colcon build \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```

检查包来源：

```bash
ros2 pkg prefix b2z1_bringup
ros2 pkg prefix unitree_go
ros2 pkg prefix z1_bringup
```

它们应指向 `/home/liu/b2z1_ros2_ws/install/`。

## 5. MuJoCo 运行流程

### 5.1 启动 Unitree MuJoCo

```bash
conda deactivate

cd /home/liu/dwbc_ws/unitree_ws/src/unitree_rl_mjlab/simulate/build

env LD_LIBRARY_PATH=/usr/local/lib:/home/liu/dwbc_ws/unitree_ws/src/unitree_rl_mjlab/simulate/mujoco/lib \
./unitree_mujoco \
  --network lo \
  --robot b2 \
  --scene /home/liu/b2z1_ros2_ws/src/b2z1_ctr/sim/mujoco/simulate/b2z1_mujoco/xmls/b2z1_ctrl_stage1.xml
```

如果没有实际可用的手柄，建议在 `unitree_rl_mjlab/simulate/config.yaml` 中设置：

```yaml
use_joystick: 0
```

### 5.2 启动 ROS 2 验证

每次打开新终端后执行：

```bash
cd /home/liu/b2z1_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

B2 单独站立：

```bash
ros2 launch b2z1_bringup \
  b2_stand_validation_sim.launch.py \
  network_interface:=lo
```

Z1 单独 waypoint：

```bash
ros2 launch b2z1_bringup \
  z1_waypoint_validation_sim.launch.py \
  network_interface:=lo
```

B2+Z1 联合动作：

```bash
ros2 launch b2z1_bringup \
  b2z1_joint_validation_sim.launch.py \
  network_interface:=lo
```

通用入口也可以通过 `validation_mode` 选择验证节点：

```bash
ros2 launch b2z1_bringup mujoco_demo.launch.py \
  network_interface:=lo \
  validation_mode:=b2z1_joint_validation
```

## 6. 真机入口

> 以下命令仅说明入口，不代表当前建议直接执行。运行前必须确认供电、机械支撑、急停、网络接口、关节零位和控制参数。

B2 固定站立示例：

```bash
ros2 launch b2z1_bringup b2_real_lowcmd_stand.launch.py
```

Z1 官方 waypoint 示例：

```bash
ros2 launch b2z1_bringup z1_real_waypoint.launch.py
```

不要在 launch 文件名前添加 `real/`。ROS 2 从包的 share 目录按文件名查找 launch 文件。

当前没有 B2+Z1 联合真机 launch。联合真机需要先实现协调器、命令超时、关节限位、状态就绪检查、人工使能和故障状态机。

## 7. Z1 状态记录和分析

官方 `z1_examples` 中包含状态记录节点，可记录：

- `/joint_states`
- `/joint_trajectory_controller/state`
- reference、feedback、error 和 output
- 位置、速度与 effort

运行：

```bash
ros2 run z1_examples record_trajectory_states.py \
  --ros-args \
  -p output_dir:=/home/liu/b2z1_ros2_ws/z1_logs
```

绘图：

```bash
python3 /home/liu/b2z1_ros2_ws/src/tools/plot_z1_trajectory_csv.py
```

## 8. 问题解决

### 8.1 MuJoCo 加载模型后出现 `free(): invalid pointer` 或 DDS 断言

已观察到的典型现象：

```text
MuJoCo version 3.3.6
Mujoco data is prepared
...
free(): invalid pointer
已中止 (核心已转储)
```

或者：

```text
dds_writecdr_impl_common: Assertion
`(wr->m_iox_pub == NULL) == (d->a.iox_chunk == NULL)' failed
已中止 (核心已转储)
```

如果 Link、Joint、Actuator 和 Sensor 列表已经完整打印，说明 MuJoCo 模型和 XML 路径已经成功加载。崩溃发生在第一次发布 DDS 状态时，原因通常是 CycloneDDS C 和 C++ 动态库来自不同安装位置：

```text
libddsc.so.0   -> /opt/ros/humble/lib/x86_64-linux-gnu/libddsc.so.0
libddscxx.so.0 -> /usr/local/lib/libddscxx.so.0
```

ROS 2 Humble 的 `libddsc` 带有 Iceoryx 支持，而 `/usr/local/lib` 中的 `libddscxx` 来自 Unitree SDK。混合加载会造成 ABI/共享内存行为不一致，最终表现为 DDS 断言或堆内存错误。

MuJoCo 终端不要 source ROS，使用以下命令固定加载 Unitree SDK 配套库：

```bash
conda deactivate

cd /home/liu/dwbc_ws/unitree_ws/src/unitree_rl_mjlab/simulate/build

env LD_LIBRARY_PATH=/usr/local/lib:/home/liu/dwbc_ws/unitree_ws/src/unitree_rl_mjlab/simulate/mujoco/lib \
./unitree_mujoco \
  --network lo \
  --robot b2 \
  --scene /home/liu/b2z1_ros2_ws/src/b2z1_ctr/sim/mujoco/simulate/b2z1_mujoco/xmls/b2z1_ctrl_stage1.xml
```

ROS 2 终端需要 source 环境，但必须把 `/usr/local/lib` 放在 ROS 库目录之前：

```bash
conda deactivate

cd /home/liu/b2z1_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

ros2 launch b2z1_bringup \
  b2z1_joint_validation_sim.launch.py \
  network_interface:=lo
```

检查仿真器实际加载的 DDS 库：

```bash
env LD_LIBRARY_PATH=/usr/local/lib:/home/liu/dwbc_ws/unitree_ws/src/unitree_rl_mjlab/simulate/mujoco/lib \
ldd /home/liu/dwbc_ws/unitree_ws/src/unitree_rl_mjlab/simulate/build/unitree_mujoco \
  | grep ddsc
```

正确结果应全部来自 `/usr/local/lib`：

```text
libddsc.so.0   => /usr/local/lib/libddsc.so.0
libddscxx.so.0 => /usr/local/lib/libddscxx.so.0
```

下面这条日志只是 loopback 网卡不支持 multicast 的提示，不是上述崩溃的原因：

```text
selected interface "lo" is not multicast-capable: disabling multicast
```

### 8.2 ROS 2 找不到 `real/...launch.py`

不要把源码中的子目录写进 `ros2 launch` 参数：

```bash
# 错误
ros2 launch b2z1_bringup real/z1_real_waypoint.launch.py

# 正确
ros2 launch b2z1_bringup z1_real_waypoint.launch.py
```

B2 真机入口同理，应使用：

```bash
ros2 launch b2z1_bringup b2_real_lowcmd_stand.launch.py
```

### 8.3 ROS 2 Python 与 Conda 冲突

如果出现 `rclpy._rclpy_pybind11` 缺失，或者错误信息中同时出现 ROS 2 Python 3.10 与 Conda Python 3.12/3.13 路径，先退出 Conda 再重新 source 和构建：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
```

构建时继续显式指定系统 Python：

```bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## 9. 与 ALORE 部署代码的关系

参考仓库：

```text
/home/liu/deploy_ws/b2z1/ALORE_Legged_Manipulator
```

ALORE 的可复用设计思想是：由同一个上层控制循环维护 B2 命令和 Z1 目标，再分别交给两个底层后端执行。

```text
统一策略/状态机
  ├── B2 -> Unitree DDS LowCmd
  └── Z1 -> 常驻 Z1 控制器
```

当前项目保留相同的职责划分，但继续使用 ROS 2 官方 Z1 链路，而不是直接复制 ALORE 的 ROS 1 + Z1 SDK 实现。后续重点借鉴：

- 等待 B2、Z1 状态都有效后再使能。
- 同一周期生成两路目标。
- 关节限位和单周期变化量限制。
- 命令 watchdog。
- `DISARMED / ARMED / RUNNING / FAULT` 状态机。
- 人工启动、停止和急停接口。

## 10. 当前边界与下一步

当前已经具备：

- B2 和 Z1 各自的真机基础执行链路。
- B2、Z1 单独 MuJoCo 验证。
- B2+Z1 同时运动的 MuJoCo 验证。
- B2 共享站立轨迹和 Z1 共享 waypoint。
- Z1 状态记录与离线绘图。

当前尚未具备：

- 统一的真机 `JointTrajectory` 输入接口。
- B2 `JointTrajectory -> LowCmd` 通用执行器。
- B2 与 Z1 的统一开始时间和进度同步。
- 真机联合控制器和安全状态机。
- 策略模型接入。
- MuJoCo 与真机硬件行为的等价性证明。

推荐开发顺序：

1. 在 MuJoCo 中稳定复现 `b2z1_joint_validation`。
2. 定义统一的 12+6 关节命令、时间戳和执行状态。
3. 将 B2 固定站立节点改造成通用轨迹执行器。
4. 将 Z1 waypoint 节点改造成受协调器控制的 action 适配器。
5. 增加限位、变化率、watchdog、人工使能和故障状态机。
6. 排除 Z1 保险丝故障原因后，先做 Z1 单机低风险验证。
7. 最后进行 B2+Z1 联合真机验证和策略接入。

## 11. 第三方来源

详细归属见 [THIRD_PARTY_NOTICES.md](./THIRD_PARTY_NOTICES.md)。主要参考包括：

- Unitree `unitree_ros2` 的 B2 lowcmd 与 motion switch 示例。
- 官方 `z1_ros2` 的硬件接口、bringup、controller 和 waypoint 示例。
- Unitree MuJoCo 通信与模型约定。
- ALORE Legged Manipulator 的双后端联合部署结构。

项目自定义代码主要负责 ROS 2 包组织、共享轨迹、验证消息、MuJoCo bridge、联合验证节点和启动入口。
