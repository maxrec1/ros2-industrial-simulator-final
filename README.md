# ROS2 Industrial Robot Simulation

A ROS2-based simulation workspace for an industrial SCARA robot, including a conveyor belt system and robot visualization.

## ROS2 Version

**ROS2 Humble Hawksbill** (Ubuntu 22.04 LTS)

---

## Installation (Ubuntu 22.04 + ROS2 Humble)

### 1. Clone the repository

```bash
git clone https://github.com/maxrec1/ros2-industrial-robot-simulation-new.git
cd ros2-industrial-robot-simulation-new
```

### 2. Install system dependencies

```bash
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-moveit \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-planners \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions \
  python3-rosdep
```

### 3. Initialize rosdep (skip if already done)

```bash
sudo rosdep init
rosdep update
```

### 4. Clone the required plugins

**IFRA_ConveyorBelt** — Gazebo conveyor belt plugin:

```bash
cd src
git clone https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt.git
cd ..
```

**IFRA_LinkAttacher** — Gazebo attach/detach plugin used by the pick-and-place cycle to grip objects with the robot end-effector:

```bash
cd src
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
cd ..
```

### 5. Install remaining ROS dependencies

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 6. Build

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Packages

### `scara_robot_pkg`
Visualization package for the SCARA robot. Contains the URDF model, mesh files (STL), RViz configuration, and a launch file to view the robot with interactive joint controls.

**Launch:**
```bash
ros2 launch scara_robot_pkg view_robot.launch.py
```

Opens RViz with the SCARA robot model and `joint_state_publisher_gui` for manually moving joints.

### `IFRA_ConveyorBelt`
Gazebo conveyor belt simulation plugin, split into three sub-packages:
- **`conveyorbelt_msgs`** — Custom ROS2 service/message definitions for controlling the belt
- **`conveyorbelt_gazebo`** — Gazebo plugin implementation
- **`ros2_conveyorbelt`** — ROS2 integration node and launch files

## Running the Simulation

```bash
ros2 launch scara_robot_pkg pick_place_pipeline.launch.py
```

## Conveyor Belt Control

### Spawn objects

Spawn **PCB** on Conveyor 1 (runs along Y axis):
```bash
ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyorbelt_gazebo" --urdf "pcb.urdf" --name "pcb1" --x 0.0 --y -0.5 --z 1.2
```

Spawn **Chip** on Conveyor 2 (runs along X axis):
```bash
ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyorbelt_gazebo" --urdf "chip.urdf" --name "chip1" --x -1.3 --y -1.0 --z 1.2
```

### Activate / Stop Conveyor 1

```bash
# Activate (power 0.0–100.0)
ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 20.0}"

# Stop
ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 0.0}"
```

### Activate / Stop Conveyor 2

```bash
# Activate (power 0.0–100.0)
ros2 service call /belt2/CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 20.0}"

# Stop
ros2 service call /belt2/CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 0.0}"
```
## Graphify
```
cd /home/maxrec/projects/test/ros2-industrial-robot-simulation/graphify-out
python3 -m http.server 8080
```
