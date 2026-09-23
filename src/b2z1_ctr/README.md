# B2Z1 ROS2 Workspace

This workspace is currently organized around two separate mainlines:

- `Real robot`: B2 and Z1 execution paths that stay close to official code
- `Simulation`: a MuJoCo validation path used to observe expected control effects

`B2Z1` combined control is intentionally **out of scope for the current mainline**.

## 1. Mainlines

### 1.1 Real-robot mainline

Purpose:

- keep the B2 real low-level path clear and usable
- keep the Z1 real trajectory-control path clear and usable

Packages mainly used by the real-robot path:

- `real/b2/b2z1_b2_lowcmd`
- `real/z1/b2z1_z1_control`
- `shared/b2/b2z1_b2_trajectories`
- `shared/z1/b2z1_z1_trajectories`
- `bringup/b2z1_bringup`
- `third_party/unitree/unitree_go`
- `third_party/unitree/unitree_api`

Real-robot launch entrypoints:

- `ros2 launch b2z1_bringup real/b2_real_lowcmd_stand.launch.py`
- `ros2 launch b2z1_bringup real/z1_real_waypoint.launch.py`

### 1.2 MuJoCo simulation mainline

Purpose:

- validate command flow before real-robot testing
- validate poses, timing, and validation behavior in MuJoCo

Packages mainly used by the simulation path:

- `sim/mujoco/b2z1_mujoco_bridge`
- `sim/mujoco/b2z1_examples`
- `sim/mujoco/simulate/b2z1_mujoco`
- `shared/common/b2z1_msgs`
- `shared/b2/b2z1_b2_trajectories`
- `shared/z1/b2z1_z1_trajectories`
- `bringup/b2z1_bringup`

Simulation launch entrypoints:

- `ros2 launch b2z1_bringup b2_stand_validation_sim.launch.py network_interface:=lo`
- `ros2 launch b2z1_bringup z1_waypoint_validation_sim.launch.py network_interface:=lo`
- internally this resolves to `launch/mujoco/*`

### 1.3 Important boundary

This repository does **not** currently implement a complete `B2Z1` combined-control mainline.

The current mainline is narrower:

- B2 real control
- Z1 real control
- MuJoCo validation of those control meanings

## 2. Current Package Structure

Workspace tree:

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

Current layering:

- `bringup/`: launch entrypoints only
- `real/`: real-robot execution backends
- `shared/`: reusable motion definitions and message interfaces
- `sim/`: MuJoCo-facing execution and validation entrypoints
- `third_party/`: vendored upstream dependencies
- `legacy/`: old combined-control experiments kept out of the mainline

### 2.1 Real backends

- `b2z1_b2_lowcmd`
  - B2 real-robot backend
  - official-style `lowcmd` execution path
  - aligned with the Unitree B2 low-level stand example

- `b2z1_z1_control`
  - Z1 real-robot backend
  - official-style `FollowJointTrajectory` execution path
  - aligned with the official `z1_ros2` waypoint example

### 2.2 Shared trajectory layers

- `b2z1_b2_trajectories`
  - shared B2 stand trajectory definition and timing

- `b2z1_z1_trajectories`
  - shared Z1 waypoint sequences for simulation-side use

### 2.3 Simulation backend

- `b2z1_mujoco_bridge`
  - MuJoCo backend
  - receives experimental MuJoCo demo ROS2 commands
  - publishes `rt/lowcmd`
  - subscribes `rt/lowstate`

- `simulate/b2z1_mujoco`
  - MuJoCo XML scene files and model assets

### 2.4 Validation and entrypoint packages

- `b2z1_examples`
  - simulation-side validation nodes
  - currently keeps `b2_stand_validation` and `z1_waypoint_validation`

- `b2z1_bringup`
  - launch entry organization
  - `launch/real`
  - `launch/mujoco`

### 2.5 Support packages

- `b2z1_msgs`
  - simulation-side ROS2 messages for the current validation path

- `unitree_go`
  - vendored official Unitree message package

- `unitree_api`
  - vendored official Unitree API package

Main package relationships:

- `b2z1_b2_lowcmd` depends on `b2z1_b2_trajectories`, `unitree_go`, and `unitree_api`
- `b2z1_z1_control` depends on `b2z1_z1_trajectories` and official `z1_bringup`
- `b2z1_mujoco_bridge` depends on `b2z1_msgs`, `b2z1_b2_lowcmd`, and `b2z1_b2_trajectories`
- `b2z1_examples` publishes experimental MuJoCo validation commands to `b2z1_mujoco_bridge`
- `b2z1_bringup` is the top-level launch entry for both real and simulation flows

## 3. Reuse From Official Reference Code

See also [THIRD_PARTY_NOTICES.md](./THIRD_PARTY_NOTICES.md).

The guiding rule for this workspace is:

- reuse official code directly when practical
- keep custom code as thin as possible when direct reuse is not enough

### 3.1 B2 references

Primary B2 reference:

- `/home/liu/dwbc_ws/unitree_ws/src/unitree_ros2/example/src/src/b2/b2_stand_example.cpp`

This reference drives:

- the `lowcmd` direction for B2 real control
- the multi-stage stand timing structure
- the stand target posture organization
- the release of conflicting motion-control mode before low-level execution

Several helper files in `b2z1_b2_lowcmd` were copied directly from official Unitree code and only minimally adjusted for package/include layout.

### 3.2 Z1 references

Primary Z1 reference root:

- `/home/liu/dwbc_ws/unitree_ws/src/z1_ros2`

The real-robot Z1 direction is aligned with:

- `z1_bringup`
- `z1_hardware_interface`
- `joint_trajectory_controller`
- `FollowJointTrajectory`
- `z1_ros2/z1_examples/z1_examples/waypoint_test.py`

The following file in this workspace was copied from the official Z1 example and only minimally adjusted for package naming:

- `b2z1_z1_control/b2z1_z1_control/z1_waypoint_test.py`

## 4. Real vs Simulation Relationship

The intended design is:

- shared motion meaning
- different execution targets

For B2, this is already fairly clean:

- shared trajectory in `b2z1_b2_trajectories`
- real execution in `b2z1_b2_lowcmd`
- MuJoCo execution in `b2z1_mujoco_bridge`

For Z1, this is partly clean:

- real execution in `b2z1_z1_control`
- shared waypoint source in `b2z1_z1_trajectories`
- MuJoCo execution still goes through a lowcmd/PD-style adapter

That means:

- the real-robot Z1 path and the simulation path now share the same official waypoint source
- execution behavior is still not identical, because the real robot uses `joint_trajectory_controller` while MuJoCo still uses a lowcmd/PD-style adapter

## 5. What MuJoCo Validation Proves

Current MuJoCo validation **does prove**:

- the package structure is workable
- the command flow is sensible
- the B2 low-level direction is reasonable
- the codebase is a practical base for later policy-control work

Current MuJoCo validation **does not fully prove**:

- identical real-robot behavior
- full equivalence to the official `z1_ros2` trajectory-controller execution
- final real-robot safety

## 6. Build

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

## 7. Real-Robot Usage

### 7.1 B2 real lowcmd

Direct launch:

```bash
ros2 launch b2z1_bringup real/b2_real_lowcmd_stand.launch.py
```

This path:

- reads `/lowstate`
- publishes `/lowcmd`
- calls `ReleaseMode()`
- executes the shared B2 stand trajectory

### 7.2 Z1 real waypoint test

Direct launch:

```bash
ros2 launch b2z1_bringup real/z1_real_waypoint.launch.py
```

## 8. MuJoCo Usage

### 8.1 Start Unitree MuJoCo

Assuming `unitree_rl_mjlab` is installed elsewhere:

```bash
cd /path/to/unitree_rl_mjlab/simulate/build
./unitree_mujoco --network lo --robot b2 --scene /home/liu/b2z1_ros2_ws/src/sim/mujoco/simulate/b2z1_mujoco/xmls/b2z1_ctrl_stage1.xml
```

Recommended setting in `unitree_rl_mjlab/simulate/config.yaml`:

- `use_joystick: 0`

### 8.2 Run the current MuJoCo validations

```bash
cd ~/b2z1_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch b2z1_bringup b2_stand_validation_sim.launch.py network_interface:=lo
```

```bash
ros2 launch b2z1_bringup z1_waypoint_validation_sim.launch.py network_interface:=lo
```

Validation targets:

- `b2_stand_validation_sim.launch.py`
  - validates the B2 stand semantics used by the real lowcmd path
- `z1_waypoint_validation_sim.launch.py`
  - validates the Z1 waypoint semantics used by the real waypoint path

Internally these resolve to the simulation launch layer under `launch/mujoco`.

## 9. Current MuJoCo-Specific Adaptations

To keep the current simulation stable, a few **simulation-only** adaptations are still kept:

- `ground_support`
  - gives B2 light support during `Z1`-only simulation

- `gentle retract`
  - smooths the Z1 return motion in simulation

These are intentional MuJoCo adaptations and should not be confused with the final real-robot control logic.
