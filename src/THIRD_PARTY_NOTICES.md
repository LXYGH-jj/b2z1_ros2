# Third-Party Notices

This repository contains or references third-party code and assets.

## 1. `unitree_ros2` references

The B2 real-robot lowcmd direction in this repository was aligned against the official Unitree B2 low-level example:
- `unitree_ros2/example/src/src/b2/b2_stand_example.cpp`

This means the following ideas are strongly reference-aligned:
- multi-stage B2 stand timing,
- B2 stand target posture organization,
- lowcmd-oriented B2 control direction.

The following helper files in `b2z1_b2_lowcmd` were copied from official Unitree reference code and only minimally adjusted for package/include layout:

- `include/b2z1_b2_lowcmd/b2/b2_motion_switch_client.hpp`
- `include/b2z1_b2_lowcmd/common/base_client.hpp`
- `include/b2z1_b2_lowcmd/common/b2_base_client.hpp`
- `include/b2z1_b2_lowcmd/common/motor_crc.h`
- `src/common/motor_crc.cpp`
- `include/b2z1_b2_lowcmd/common/patch.hpp`
- `include/b2z1_b2_lowcmd/common/time_tools.hpp`
- `include/b2z1_b2_lowcmd/common/ut_errror.hpp`

Vendored official ROS2 dependency packages copied into this workspace:

- `src/third_party/unitree/unitree_go`
- `src/third_party/unitree/unitree_api`

## 2. `z1_ros2` references

The Z1 command waypoints, joint ordering, and intended real-robot control direction in this repository follow the official `z1_ros2` stack as closely as practical.

Reference source:
- `z1_ros2/z1_examples/z1_examples/waypoint_test.py`
- `z1_ros2/z1_bringup`
- `z1_ros2/z1_hardware_interface`

The following file in this workspace was copied from the official `z1_ros2` example and only minimally adjusted for package naming:

- `src/real/z1/b2z1_z1_control/b2z1_z1_control/z1_waypoint_test.py`

## 3. MuJoCo model assets

Path in this repository:
- `src/sim/mujoco/simulate/b2z1_mujoco`

These assets were copied from earlier B2Z1 model work and then adapted for Unitree MuJoCo loopback validation.
They should be treated as imported model assets plus local adaptation work, not as a fully original model authored from scratch in this repository.


# 第三方声明

本仓库包含或引用了第三方代码和资源。

## 1. `unitree_ros2` 引用

本仓库中的 B2 真机 lowcmd 控制方向主要对齐于官方宇树 B2 底层示例：
- `unitree_ros2/example/src/src/b2/b2_stand_example.cpp`

也就是说，本仓库中的以下内容与官方参考强相关：
- B2 多阶段起立时序，
- B2 起立目标姿态组织，
- 面向 lowcmd 的 B2 控制方向。

`b2z1_b2_lowcmd` 中以下辅助文件直接复制自官方参考代码，仅做了最小化的包结构和 include 路径调整：

- `include/b2z1_b2_lowcmd/b2/b2_motion_switch_client.hpp`
- `include/b2z1_b2_lowcmd/common/base_client.hpp`
- `include/b2z1_b2_lowcmd/common/b2_base_client.hpp`
- `include/b2z1_b2_lowcmd/common/motor_crc.h`
- `src/common/motor_crc.cpp`
- `include/b2z1_b2_lowcmd/common/patch.hpp`
- `include/b2z1_b2_lowcmd/common/time_tools.hpp`
- `include/b2z1_b2_lowcmd/common/ut_errror.hpp`

本工作空间中一并 vendored 进来的官方 ROS2 依赖包有：

- `src/third_party/unitree/unitree_go`
- `src/third_party/unitree/unitree_api`

## 2. `z1_ros2` 引用

本仓库中的 Z1 命令关键点、关节顺序以及预期真机控制方向尽可能紧密地遵循官方 `z1_ros2` 堆栈。

参考来源：
- `z1_ros2/z1_examples/z1_examples/waypoint_test.py`
- `z1_ros2/z1_bringup`
- `z1_ros2/z1_hardware_interface`

## 3. MuJoCo 模型资源

在本仓库中的路径：
- `src/sim/mujoco/simulate/b2z1_mujoco`

这些资源是从早期的 B2Z1 模型工作中复制而来，随后经过调整以适用于宇树 MuJoCo 回路验证。
它们应被视为导入的模型资源加上本地适配工作，而非本仓库从头创作的完全原创模型。
