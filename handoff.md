# Handoff — bobby MoveIt + multi-robot Gazebo

Date: 2026-06-16

## Environment / where things live

- **Real workspace** (mounted into the container as `/ros2_ws`):
  `~/Schreibtisch/ros2-industrial-robot-simulation-new-main./ros2-industrial-robot-simulation-new-main`
  (The `~/Downloads/...` copy is a trimmed duplicate the IDE sometimes opens — do **not** work there.)
- **Container**: `ros2_humble` (image `osrf/ros:humble-desktop-full`) → ROS 2 Humble / Ubuntu 22.04 / **Gazebo Classic**. Host is jazzy — do not build on host.
  ```bash
  xhost +local:root
  docker start ros2_humble
  docker exec -it ros2_humble bash      # workspace at /ros2_ws
  source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
  ```
- **Extra apt packages installed in the container** (NOT in any Dockerfile yet — reinstall if the container is recreated):
  `ros-humble-ros2-controllers ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster ros-humble-gazebo-ros2-control ros-humble-ros2controlcli`

## Two robots

- **scara_robot_pkg** — 4-DOF SCARA (`Joint_1..Joint_4`). Maintained upstream by Max. Has a working `scara_moveit_config` + Gazebo pick-place pipeline.
- **bobby** — separate 6R arm (`joint_1..joint_6`) + parallel gripper (`finger_left_joint` prismatic, `finger_right_joint` mimic ×-1) + `TCP`. Victor's robot. Reworked finger-gripper + TCP.

## Status: DONE and on git (`main`, VictorSwekis fork)

`bobby_moveit_config` — standalone MoveIt config, **verified working** in RViz:
```bash
ros2 launch bobby_moveit_config demo.launch.py     # plan & execute, gripper open/close, TCP frame all OK
```
Post-generation fixes already committed: SRDF `wolrd`→`world` typo; `joint_limits.yaml` integer→double (move_group aborts on integer limit params).

This is the last clean pushed commit. **The multi-robot work below is NOT committed yet** (work in progress, currently broken).

## Status: IN PROGRESS (local only) — bobby movable + MoveIt inside the combined Gazebo scene

Goal: in `scara_robot_pkg/launch/scara_conveyor_gazebo.launch.py`, spawn bobby **movable** (was `<static>`), driven by MoveIt (plan & execute), namespaced `/bobby`, alongside SCARA + conveyors.

### New / changed files (all in the Schreibtisch copy)

- `src/bobby/scripts/gen_bobby_gazebo.py` — generator. Reads `bobby.urdf` + the standalone SRDF and writes:
  - `src/bobby/urdf/bobby_gazebo.urdf` — **link names prefixed `bobby_`** (joints left as-is; they don't clash with SCARA's `Joint_*`), a `world`→`bobby_base_link` fixed joint anchoring it on the pedestal at `(0, 1.5, 0.8)`, a `gazebo_ros2_control/GazeboSystem` `<ros2_control>` block, and the `libgazebo_ros2_control.so` plugin in namespace `/bobby`. XML comments are stripped (the SolidWorks header comment with `@`/`http://` broke gazebo_ros2_control's `--param robot_description:=...` CLI rule and aborted gzserver).
  - `src/bobby_moveit_config_gazebo/config/bobby_gazebo.srdf` — same SRDF with link refs prefixed and the `virtual_joint` removed (the URDF now owns the real `world` root link).
  - **Re-run after editing bobby.urdf:** `python3 src/bobby/scripts/gen_bobby_gazebo.py`
- `src/bobby_moveit_config_gazebo/` — copy of `bobby_moveit_config`, renamed (package.xml + CMakeLists). `config/ros2_controllers.yaml` rewritten with `/**/` wildcard node keys so params match the namespaced `/bobby/controller_manager`.
- `src/scara_robot_pkg/launch/scara_conveyor_gazebo.launch.py` — bobby section rewritten: namespaced RSP, non-static spawn from `/bobby/robot_description`, `/bobby` controller spawners (joint_state_broadcaster + arm_controller + gripper_controller), bobby `move_group` (MoveItConfigsBuilder, ns `/bobby`), bobby RViz (under `launch_rviz:=True`). The old SCARA `rviz_node` is no longer in the returned LaunchDescription (RAM).

Build: `colcon build --symlink-install` (in container). All headless checks pass: `check_urdf`, MoveItConfigsBuilder load, `generate_launch_description()`.

Launch:
```bash
ros2 launch scara_robot_pkg scara_conveyor_gazebo.launch.py launch_rviz:=True
```

### What works now

- gzserver no longer crashes (comment-strip fix).
- bobby's controllers load and are **active and correct**:
  `ros2 control list_controllers -c /bobby/controller_manager` → `arm_controller`, `gripper_controller`, `joint_state_broadcaster` all active; hardware interfaces are bobby's `joint_1..6` + `finger_left_joint`.
- `/bobby/joint_state_broadcaster` correctly publishes bobby joints to `/bobby/joint_states`.

### THE REMAINING BUG (blocks planning/execution)

bobby's `move_group` spams:
```
[moveit_robot_model]: Joint 'Joint_1'..'Joint_4' not found in model 'bobby_v3'
```
and you cannot Plan & Execute (current robot state never becomes valid).

Diagnosis so far:
- `/bobby/joint_states` has **2 publishers**. One is bobby's broadcaster (correct, `joint_1..6`). The **second emits SCARA's `Joint_1..Joint_4`** — i.e. SCARA's joint states are leaking onto bobby's namespaced topic, and bobby's move_group ingests them → "joint not found".
- `ros2 node list` shows **two** nodes named `/bobby/joint_state_broadcaster` and a stray `/bobby/arm_trajectory_controller` (that is a SCARA controller name). So a SCARA controller/broadcaster appears to be living in the `/bobby` namespace.
- `/joint_states` (global) appeared to have 0 publishers; SCARA's `arm_trajectory_controller` is `unconfigured` on `/controller_manager`.
- The move_group remap `('/joint_states', '/bobby/joint_states')` is applied but does not help, because the polluting publisher is already on `/bobby/joint_states`.

Root cause not yet pinned: why does a SCARA joint_state_broadcaster (Joint_1..4) end up publishing on `/bobby/joint_states`? There is no explicit joint_states remap in the SCARA launch/xacro.

### Next steps / hypotheses to try

1. **Pin the rogue publisher.** With the stack running:
   `ros2 node info` on each `/bobby/joint_state_broadcaster` (disambiguate the duplicate), and check the SCARA `joint_state_broadcaster_spawner` / `arm_controller_spawner` definitions in the launch — confirm their target `-c /controller_manager` and that they aren't inheriting the `/bobby` namespace or a node-name collision (`joint_state_broadcaster` name is shared by both robots).
2. **Most promising fix — give bobby's controllers unique names** (`bobby_joint_state_broadcaster`, `bobby_arm_controller`, `bobby_gripper_controller`) in `bobby_moveit_config_gazebo/config/ros2_controllers.yaml` + the launch spawners + `moveit_controllers.yaml`. Unique names remove the cross-robot collision that is likely causing the `/bobby` duplicate.
3. Double-check the `/**/` wildcard in bobby's `ros2_controllers.yaml` isn't bleeding params across both controller managers; if suspicious, switch to explicit `/bobby/...` node keys.
4. SCARA's `arm_trajectory_controller` being `unconfigured` may be a pre-existing SCARA issue; verify SCARA still works on its own (`ros2 launch scara_robot_pkg scara_conveyor_gazebo.launch.py` on the upstream version) to separate concerns.

### Caveats

- **RAM/OOM**: Gazebo + 2 robots + move_group + RViz is heavy; the container has OOM-died before (exit 137). Test with `launch_rviz:=False` first.
- `bobby_moveit_config_gazebo` files created from the host are victor-owned; `bobby_moveit_config` (Setup-Assistant generated) is **root-owned** — edit those via `docker exec`.
- In RViz MotionPlanning you must set **Move Group Namespace = `/bobby`**.

### Handy diagnostics
```bash
ros2 control list_controllers -c /bobby/controller_manager
ros2 control list_hardware_interfaces -c /bobby/controller_manager
ros2 topic info /bobby/joint_states --verbose
ros2 topic echo /bobby/joint_states --once
ros2 node list | grep -iE 'broadcaster|controller'
```
