# ROS 2 Development Portfolio: SMART Lab Preparation

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-WSL2_Ubuntu_24.04-orange?style=for-the-badge&logo=ubuntu&logoColor=white)

## Project Overview
This repository documents a progressive learning path in **ROS 2 Jazzy**, specifically tailored to the technical requirements of the **SMART Lab**. The primary objective is to build competency in **Multi-Agent Systems**, **Motion Capture integration**, and **Networked Perception** to support research in autonomous aerial robotics.

The project is structured into "Phases," evolving from single-agent communication to complex fleet orchestration and visualization.

## Technical Stack
* **Framework:** ROS 2 Jazzy Jalisco
* **Language:** Python 3.12
* **Environment:** Ubuntu 24.04 via WSL2 (Windows Subsystem for Linux)
* **Key Libraries:** `rclpy`, `std_srvs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`
* **Tools:** VS Code, Colcon, Rqt, Rviz2

---

## Repository Structure

### `smart_lab_phase1`: Fundamentals & Safety Architecture
*Focus: Node communication, Quality of Service (QoS), and asynchronous service calls.*

* **`drone_hardware`**: Simulates sensor streams (IMU data) with injected noise to mimic real-world hardware instability. Acts as a **Service Server** for emergency commands.
* **`flight_computer`**: Monitors sensor data for "Critical Tilt" events. Acts as a **Service Client**, triggering an asynchronous "Kill Switch" to disarm the hardware when crash thresholds are exceeded.

### `smart_lab_phase2`: Fleet Management & Spatial Awareness
*Focus: Multi-agent orchestration, Namespacing, and Coordinate Transforms (TF2).*

* **Fleet Orchestration**: Utilizes `fleet.launch.py` to spawn multiple instances of the drone stack (Alpha, Bravo) with isolated namespaces to prevent topic collision.
* **`mocap_simulator`**: A specialized node acting as a centralized Vicon/Motion Capture system. It broadcasts **TF2 transforms**, placing the agents (`/alpha`, `/bravo`) into a shared 3D coordinate frame (`/world`) for visualization.

### `smart_lab_phase3`: Visualization & Perception (Active)
*Focus: Rviz2 Integration, Marker Arrays, and Visual Debugging.*

* **`mocap_simulator`**: Broadcasts the "Ground Truth" TF2 coordinate frames for the fleet, anchoring them to the `world` frame.
* **`drone_hardware` (v3)**: Enhanced hardware node that publishes `visualization_msgs/Marker` arrows to visualize invisible sensor forces (Gravity/Acceleration) in real-time.
* **`fleet.launch.py` (v3)**: Orchestrates the full simulation: Mocap God-Node + Alpha Drone + Bravo Drone.

---

## Roadmap & Progress

### Phase 1: The "Kill Switch" Architecture
- [x] Workspace configuration and Colcon build workflows.
- [x] Publisher/Subscriber implementation (IMU Sensor streams).
- [x] Custom Service implementation (`std_srvs/SetBool`) for remote hardware disarming.
- [x] Asynchronous handling of service requests to prevent node blocking.

### Phase 2: The Fleet Commander
- [x] **Namespacing:** Refactoring nodes to support dynamic naming (`/alpha/imu` vs `/bravo/imu`).
- [x] **Launch Systems:** Creating Python-based launch files to orchestrate multi-node fleets.
- [x] **Log Management:** Utilizing `rqt_console` and `grep` filtering for debugging parallel processes.
- [x] **TF2 Integration:** Broadcasting dynamic coordinate frames for multiple agents.

### Phase 3: Perception & Interaction (Current)
- [x] **Package Setup:** Established `smart_lab_phase3` with dependencies for `visualization_msgs` and `tf2_ros`.
- [x] **The "God Node":** Created `mocap_simulator` to publish ground-truth TF frames.
- [x] **Basic Visualization:** Configured Rviz2 to track the `world` -> `alpha` / `bravo` tree.
- [x] **Sensor Visualization (alpha drone):** Visualizing real-time IMU gravity vectors using Rviz Markers.
- [x] **Multi-Agent Visualization:** Extending visual markers to the full fleet (alpha drone and beta drone).

### Future Goals
- [ ] **Sensor Fusion:** Integrating multiple data sources.
- [ ] **Simulation:** Potential integration with Gazebo/Ignition for physics-based testing.

---

## Usage Instructions

### 1. Build the Workspace
```bash
# cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch smart_lab_phase3 fleet.launch.py

# ros2 launch smart_lab_phase3 fleet.launch.py | grep "alpha"
```