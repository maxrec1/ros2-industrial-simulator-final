# ROS2 Industrial Robot Simulation

A ROS2-based simulation workspace for an industrial SCARA robot, including a conveyor belt system and robot visualization.

## ROS2 Version

**ROS2 Humble Hawksbill** (Ubuntu 22.04 LTS)

---

## Installation (Ubuntu 22.04 + ROS2 Humble) [Alternative: Docker Container Instructions Below]

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

# Anleitung: ROS2 SCARA-Simulation in Docker starten

Diese Anleitung beschreibt, wie die Simulation **in einem Docker-Container** auf einem
Linux-Rechner gestartet wird (genauso wie in der Entwicklungsumgebung aufgesetzt).

- **ROS2-Distribution:** Humble
- **Docker-Image:** `osrf/ros:humble-desktop-full` (öffentlich, wird automatisch geladen)
- **Robotermodell:** SCARA + Förderbänder (Gazebo / RViz / MoveIt)

---

## 0. Voraussetzungen

- **Linux-Desktop** (getestet unter Ubuntu 22.04) mit grafischer Oberfläche (X11).
  Gazebo und RViz sind GUI-Anwendungen und brauchen eine Anzeige.
- **Docker** installiert. Falls noch nicht vorhanden:

  ```bash
  sudo apt update
  sudo apt install -y docker.io
  sudo systemctl enable --now docker
  # eigenen Benutzer der docker-Gruppe hinzufügen (danach neu einloggen):
  sudo usermod -aG docker $USER
  ```

  Neu einloggen (oder `newgrp docker`), damit `docker` ohne `sudo` läuft.

- **Projektordner** auf dem Rechner liegen haben, z. B. nach dem Entpacken:
  `~/ros2-industrial-simulator-final`
  (enthält den Ordner `src/` mit allen Paketen inkl. der Plugins
  `IFRA_ConveyorBelt` und `IFRA_LinkAttacher` — diese sind bereits enthalten,
  müssen **nicht** extra geklont werden).

---

## 1. X11-Zugriff für Docker freigeben

Einmal pro Sitzung auf dem Host ausführen (erlaubt dem Container, Fenster anzuzeigen):

```bash
xhost +local:docker
```

---

## 2. Container starten

`PFAD_ZUM_PROJEKT` durch den echten Pfad des Projektordners ersetzen.

```bash
docker run -it --rm \
  --name ros2_humble \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --device /dev/dri \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume PFAD_ZUM_PROJEKT:/ros2_ws \
  --network host \
  --workdir /ros2_ws \
  osrf/ros:humble-desktop-full \
  bash
```

Beispiel mit konkretem Pfad:

```bash
  --volume ~/ros2-industrial-simulator-final:/ros2_ws \
```

Erklärung der wichtigsten Optionen:

| Option | Zweck |
|---|---|
| `--device /dev/dri` | Hardware-Grafik (Intel/AMD) für Gazebo. Bei NVIDIA siehe Hinweis unten. |
| `--volume ...:/ros2_ws` | Projektordner wird im Container unter `/ros2_ws` eingehängt |
| `--network host` | ROS2-Kommunikation / Anzeige ohne Port-Mapping |
| `--env DISPLAY` + X11-Volume | GUI-Fenster auf dem Host anzeigen |

Nach dem Befehl befindet man sich **im Container** in `/ros2_ws`.

---

## 3. Im Container: Workspace bauen

Die mitgelieferten Ordner `build/`, `install/`, `log/` wurden ggf. auf einem
anderen Rechner gebaut — daher im Container einmal **sauber neu bauen**:

```bash
# (im Container, in /ros2_ws)
rm -rf build install log

source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build
source install/setup.bash
```

> Der erste Build dauert einige Minuten.

---

## 4. Simulation starten

```bash
# (im Container, nach dem Sourcen)
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch scara_robot_pkg pick_place_pipeline.launch.py
```

Es öffnen sich Gazebo (Simulation) und RViz (Visualisierung).

---

## 5. Förderbänder steuern (optional, in weiterem Terminal im Container)

Zweites Terminal im **laufenden** Container öffnen:

```bash
docker exec -it ros2_humble bash
# dann im Container:
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

Objekt auf Band 1 (PCB) spawnen:

```bash
ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyorbelt_gazebo" \
  --urdf "pcb.urdf" --name "pcb1" --x 0.0 --y -0.5 --z 1.2
```

Band 1 starten / stoppen:

```bash
ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 20.0}"
ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 0.0}"
```

Band 2:

```bash
ros2 service call /belt2/CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 20.0}"
ros2 service call /belt2/CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 0.0}"
```

---

## 6. Beenden

- Simulation: `Ctrl+C` im Launch-Terminal
- Container verlassen: `exit` (durch `--rm` wird der Container automatisch entfernt;
  der Build unter `/ros2_ws` bleibt erhalten, da er im Projektordner liegt)

---

## Hinweise / Fehlerbehebung

- **Kein Bild / „cannot open display"**: `xhost +local:docker` auf dem Host vergessen.
- **NVIDIA-GPU**: `--device /dev/dri` weglassen und stattdessen das
  NVIDIA Container Toolkit installieren, dann `--gpus all` ergänzen. Alternativ
  reine Software-Grafik erzwingen (langsam, aber funktioniert überall):
  `--env LIBGL_ALWAYS_SOFTWARE=1` hinzufügen.
- **Image-Download**: Beim ersten `docker run` wird `osrf/ros:humble-desktop-full`
  (~5 GB) automatisch heruntergeladen. Internetverbindung nötig.
- **Build schlägt fehl**: sicherstellen, dass `src/IFRA_ConveyorBelt` und
  `src/IFRA_LinkAttacher` vorhanden sind.
