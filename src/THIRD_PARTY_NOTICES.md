# Third-Party Notices

This repository contains or references third-party code and assets.

## 1. `unitree_api`

Path in this repository:
- `src/unitree_api`

Origin:
- copied from the official Unitree ROS2 workspace layout under:
  - `unitree_ros2/cyclonedds_ws/src/unitree/unitree_api`

Purpose in this repository:
- retained as a vendored dependency so the B2 ROS2 bridge can be built reproducibly inside this workspace.

Upstream attribution visible in package metadata:
- maintainer: `Unitree <unitree@unitree.com>`
- package license field: `BSD 3-Clause License`

This repository does not claim authorship of `src/unitree_api`.

## 2. `unitree_ros2` references

The following parts of this repository were written by strongly reusing or adapting official Unitree reference code:
- `src/b2z1_b2_bridge/include/b2z1_b2_bridge/ros2_b2_sport_client.hpp`
- `src/b2z1_b2_bridge/src/ros2_b2_sport_client.cpp`

Reference source:
- `unitree_ros2/example/src/include/common/ros2_b2_sport_client.h`
- `unitree_ros2/example/src/src/common/ros2_b2_sport_client.cpp`
- `unitree_ros2/example/src/src/b2/b2_stand_example.cpp`

## 3. `z1_ros2` references

The Z1 command waypoints, joint ordering, and real-robot control direction in this repository follow the official `z1_ros2` stack as closely as practical.

Reference source:
- `z1_ros2/z1_examples/z1_examples/waypoint_test.py`
- `z1_ros2/z1_bringup`
- `z1_ros2/z1_hardware_interface`

## 4. MuJoCo model assets

Path in this repository:
- `src/simulate/b2z1_mujoco`

These assets were copied from earlier B2Z1 model work and then adapted for Unitree MuJoCo loopback validation.
They should be treated as imported model assets plus local adaptation work, not as a fully original model authored from scratch in this repository.


# 第三方声明

本仓库包含或引用了第三方代码和资源。

## 1. `unitree_api`

在本仓库中的路径：
- `src/unitree_api`

来源：
- 复制自官方宇树 ROS2 工作空间布局下的：
  - `unitree_ros2/cyclonedds_ws/src/unitree/unitree_api`

在本仓库中的用途：
- 作为供应商依赖项保留，以便在此工作空间内可重现地构建 B2 ROS2 桥接器。

在包元数据中可见的上游归属信息：
- 维护者：`Unitree <unitree@unitree.com>`
- 包许可证字段：`BSD 3-Clause License`

本仓库不声称拥有 `src/unitree_api` 的著作权。

## 2. `unitree_ros2` 引用

本仓库的以下部分通过强力复用或适配官方宇树参考代码编写而成：
- `src/b2z1_b2_bridge/include/b2z1_b2_bridge/ros2_b2_sport_client.hpp`
- `src/b2z1_b2_bridge/src/ros2_b2_sport_client.cpp`

参考来源：
- `unitree_ros2/example/src/include/common/ros2_b2_sport_client.h`
- `unitree_ros2/example/src/src/common/ros2_b2_sport_client.cpp`
- `unitree_ros2/example/src/src/b2/b2_stand_example.cpp`

## 3. `z1_ros2` 引用

本仓库中的 Z1 命令关键点、关节顺序以及真机控制方向尽可能紧密地遵循官方 `z1_ros2` 堆栈。

参考来源：
- `z1_ros2/z1_examples/z1_examples/waypoint_test.py`
- `z1_ros2/z1_bringup`
- `z1_ros2/z1_hardware_interface`

## 4. MuJoCo 模型资源

在本仓库中的路径：
- `src/simulate/b2z1_mujoco`

这些资源是从早期的 B2Z1 模型工作中复制而来，随后经过调整以适用于宇树 MuJoCo 回路验证。
它们应被视为导入的模型资源加上本地适配工作，而非本仓库从头创作的完全原创模型。