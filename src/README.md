# B2Z1 ROS2 Workspace

This workspace contains a lightweight ROS2 control skeleton for Unitree B2 + Z1, plus a MuJoCo validation path used to verify command flow, poses, and demo behavior before real-robot testing.

The current repository intentionally separates two concerns:

- MuJoCo validation: fast iteration for B2-only and Z1-only demos.
- Real robot integration: reuse official Unitree code paths as much as possible.

## 1. Repository Scope

The main packages in this workspace are:

- `b2z1_msgs`: custom ROS2 message definitions.
- `b2z1_b2_bridge`: B2 command bridge.
- `b2z1_coordinator`: optional coordinator node for future unified command routing.
- `b2z1_examples`: demo scripts for B2-only and Z1-only validation.
- `b2z1_mujoco_bridge`: bridge from ROS2 demo commands to MuJoCo low-level commands.
- `b2z1_bringup`: launch files for MuJoCo validation demos.
- `simulate/b2z1_mujoco`: B2Z1 MuJoCo scene and assets.
- `unitree_api`: vendored dependency copied from the official Unitree ROS2 workspace and kept here for reproducible local builds.

## 2. What Was Reused From Reference Code

See also [THIRD_PARTY_NOTICES.md](./THIRD_PARTY_NOTICES.md) for vendored dependency and attribution notes.

The project tries to follow the principle: reuse official/reference implementations whenever practical, and only add custom logic where the combined B2+Z1 MuJoCo path requires it.

### 2.1 Directly copied or strongly adapted from `unitree_ros2`

Reference root:
- `/home/liu/dwbc_ws/unitree_ws/src/unitree_ros2`

Main reused pieces:
- `b2z1_b2_bridge/include/b2z1_b2_bridge/ros2_b2_sport_client.hpp`
- `b2z1_b2_bridge/src/ros2_b2_sport_client.cpp`

These files are based on the official B2 sport client utilities from:
- `unitree_ros2/example/src/include/common/ros2_b2_sport_client.h`
- `unitree_ros2/example/src/src/common/ros2_b2_sport_client.cpp`

The B2 mode semantics also follow the official examples:
- `balance_stand`
- `stand_down`
- `damp`
- `move`

B2 stand timing and stage structure in the MuJoCo bridge were aligned against:
- `unitree_ros2/example/src/src/b2/b2_stand_example.cpp`

### 2.2 Directly copied or strongly adapted from `z1_ros2`

Reference root:
- `/home/liu/dwbc_ws/unitree_ws/src/z1_ros2`

Main reused ideas:
- Z1 joint naming and ordering are kept consistent with `z1_ros2`.
- Z1 pose waypoints used in demos are aligned with `z1_ros2/z1_examples/z1_examples/waypoint_test.py` wherever possible:
  - `home`
  - `reach`
  - `wrist pose`

For real robot usage, the intended official path remains:
- `z1_bringup`
- `z1_hardware_interface`
- `joint_trajectory_controller`
- `FollowJointTrajectory`

### 2.3 Directly copied external model assets

MuJoCo model resources in:
- `simulate/b2z1_mujoco`

These were copied from earlier B2Z1 model work and then adapted for Unitree MuJoCo loopback validation. In particular:
- the base B2Z1 model/assets were copied from an earlier training/legacy model source,
- then adjusted to satisfy Unitree MuJoCo control expectations,
- and further aligned against the Go2 scene template conventions where needed.

### 2.4 Vendored dependency retained in this repository

The following package is copied into this workspace intentionally so the workspace can be built reproducibly without asking every user to reconstruct the same subdirectory layout by hand:
- `src/unitree_api`

Source lineage:
- copied from `unitree_ros2/cyclonedds_ws/src/unitree/unitree_api`
- upstream package metadata declares maintainer `Unitree <unitree@unitree.com>` and license `BSD 3-Clause License`

This repository does not claim authorship of `unitree_api`; it is retained here as a vendored dependency.

### 2.4 Newly added custom code in this workspace

The following parts are custom to this workspace:
- `b2z1_msgs`
- `b2z1_coordinator`
- `b2z1_examples`
- `b2z1_mujoco_bridge`
- `b2z1_bringup`

These are not official upstream files. They were added to:
- organize the workspace into ROS2 packages,
- connect demo scripts to MuJoCo loopback control,
- provide a future base for higher-level policy/control work.

## 3. Important Boundary: MuJoCo Path vs Real Robot Path

### MuJoCo validation path

The current MuJoCo demos execute roughly as:

- `b2z1_examples`
- `b2z1_mujoco_bridge`
- `rt/lowcmd`
- `unitree_mujoco`

For Z1, this means MuJoCo currently uses direct low-level PD-style command execution.

### Official real robot Z1 path

The official validated Z1 stack in `z1_ros2` executes through:

- trajectory/action command
- `joint_trajectory_controller`
- hardware interface
- real robot

This is the reason the MuJoCo demo path is **not identical** to `z1_ros2` execution, even when the waypoint poses are aligned.

### What this means

Current MuJoCo validation **does prove**:
- workspace/package structure is sound,
- command definitions are reasonable,
- demo logic and pose organization are usable,
- the codebase is a practical base for later policy-control work.

Current MuJoCo validation **does not fully prove**:
- identical behavior on the real robot,
- full equivalence to `z1_ros2` trajectory-controller execution,
- final real-robot safety.

## 3.5 Upload policy for this repository

This repository keeps a small amount of copied upstream code on purpose when it improves reproducibility. In particular:
- `src/unitree_api` is intentionally committed as a vendored dependency
- upstream references are documented in this README and in `THIRD_PARTY_NOTICES.md`
- custom packages in this repository remain the primary authored content

## 4. MuJoCo-Only Workspace Dependencies

To run the current MuJoCo demos on another computer, you minimally need:

From this workspace:
- `b2z1_msgs`
- `b2z1_examples`
- `b2z1_mujoco_bridge`
- `b2z1_bringup`
- `simulate/b2z1_mujoco`

External dependency:
- a separately installed `unitree_rl_mjlab` providing `unitree_mujoco`

Optional for real robot work:
- `b2z1_b2_bridge`
- `b2z1_coordinator`
- `unitree_api`
- official `z1_ros2` packages

## 5. Build Instructions

### 5.1 General build

```bash
cd ~/b2z1_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

If conda is active, it is safer to leave it first:

```bash
conda deactivate
```

## 6. MuJoCo Usage

### 6.1 Start Unitree MuJoCo

Assuming `unitree_rl_mjlab` is installed elsewhere on the machine:

```bash
cd /path/to/unitree_rl_mjlab/simulate/build
./unitree_mujoco --network lo --robot b2 --scene /home/liu/b2z1_ros2_ws/src/simulate/b2z1_mujoco/xmls/b2z1_ctrl_stage1.xml
```

Note:
- in `unitree_rl_mjlab/simulate/config.yaml`, `use_joystick: 0` is recommended unless a joystick is actually available.

### 6.2 Run B2-only stand demo

```bash
cd ~/b2z1_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch b2z1_bringup b2_only_stand.launch.py network_interface:=lo
```

### 6.3 Run B2 stand-then-down demo

```bash
ros2 launch b2z1_bringup b2_only_stand_then_down.launch.py network_interface:=lo
```

### 6.4 Run Z1-only reach demo

```bash
ros2 launch b2z1_bringup z1_only_reach.launch.py network_interface:=lo
```

## 7. Real Robot Usage

### 7.1 B2 real robot direction

For real B2 usage, prefer the official Unitree path and semantics:
- official `unitree_ros2` examples,
- `unitree_api`,
- sport client semantics.

This workspace contains:
- `b2z1_b2_bridge`

which is intended as a thin bridge layer, not a replacement for official B2 communication semantics.

### 7.2 Z1 real robot direction

For real Z1 usage, prefer the official `z1_ros2` stack:
- `z1_bringup`
- `z1_hardware_interface`
- `joint_trajectory_controller`
- `FollowJointTrajectory`

The current MuJoCo demos should be treated as:
- validation of structure,
- validation of waypoint choices,
- validation of high-level control organization,

rather than final proof of real robot behavior.

## 8. Why This Workspace Is Still Useful For Future Policy Control

This workspace is already a reasonable base layer for later policy-control work because it provides:
- explicit ROS2 package boundaries,
- a message layer (`b2z1_msgs`),
- demo entry points (`b2z1_examples`),
- a MuJoCo execution bridge (`b2z1_mujoco_bridge`),
- launch organization (`b2z1_bringup`).

A future policy can therefore be added above this layer without rewriting the whole workspace.

## 9. Known MuJoCo-Specific Adaptations

The code tries to minimize custom behavior, but a few MuJoCo-only adaptations remain for stability:
- `ground_support` for B2 during Z1-only validation,
- a `gentle retract` stage in the Z1-only demo,
- small protective adaptations in the B2 MuJoCo stand path to avoid instability in the combined B2+Z1 model.

These are intentional simulation adaptations and should not be confused with the final real-robot execution chain.
