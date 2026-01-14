# Two-Link Robotic Arm (ROS 2)

This repository provides a complete ROS 2 workspace for a two-degree-of-freedom (2-DOF) planar robotic arm. It includes an inverse kinematics (IK) node implemented in Python, a URDF model for visualization, and an RViz launch configuration. The project is intended for educational use and as a foundation for further work in trajectory generation, control, and simulation.

---

## Overview

The workspace contains two ROS 2 packages:

- `arm_kinematics`: Python package implementing inverse kinematics for a 2-link planar arm.
- `arm_description`: URDF model, robot_state_publisher configuration, and RViz visualization launch.

The manipulator is modeled as a planar arm with two revolute joints:
- Link 1 length: 0.30 m
- Link 2 length: 0.20 m

---

## Features

### Inverse Kinematics Node (Python)
The IK node:
- Solves inverse kinematics for a 2-link planar arm.
- Publishes joint states for downstream visualization/control.
- Subscribes to a target point topic (`geometry_msgs/msg/Point`) for desired Cartesian goals.
- Runs periodically on a 1-second timer for convenient testing.

Run:
```bash
ros2 run arm_kinematics two_link_ik
```

### URDF Model
The URDF model is located at:
- `arm_description/urdf/two_link.urdf`

It includes:
- Base link
- Link 1 (0.30 m)
- Link 2 (0.20 m)
- Two revolute joints

### RViz Visualization
A launch file is provided to visualize the robot model and control joint angles via GUI sliders:

```bash
ros2 launch arm_description display_two_link.launch.py
```

This launches:
- RViz2
- `joint_state_publisher_gui`
- `robot_state_publisher`

---

## Repository Structure

```
two_link_arm_ros2/
├── arm_kinematics/
│   ├── two_link_ik.py
│   ├── setup.py
│   └── package.xml
├── arm_description/
│   ├── urdf/
│   │   └── two_link.urdf
│   ├── launch/
│   │   └── display_two_link.launch.py
│   └── package.xml
├── build/    (ignored)
├── install/  (ignored)
├── log/      (ignored)
└── README.md
```

---

## Requirements

- ROS 2 (Jazzy or any recent ROS 2 distribution)
- Python 3
- `rclpy`
- `numpy` (if used by the IK implementation)
- Message packages:
  - `geometry_msgs`
  - `sensor_msgs`

---

## Installation

Clone the repository and enter the workspace directory:

```bash
git clone https://github.com/Lily-Evan/two_link_arm_ros2.git
cd two_link_arm_ros2
```

Build and source the workspace:

```bash
colcon build
source install/setup.bash
```

---

## Using the IK Node

Run the inverse kinematics node:

```bash
ros2 run arm_kinematics two_link_ik
```

Publish a target point (example):

```bash
ros2 topic pub /target_point geometry_msgs/msg/Point "{x: 0.2, y: 0.1, z: 0.0}"
```

Notes:
- Targets must be within the reachable workspace of the arm:
  - \(r \le l_1 + l_2\)
  - \(r \ge |l_1 - l_2|\)
- If the target is out of reach, the node should either reject it or saturate/clip the solution (implementation-dependent).

---

## Visualization

Launch the RViz visualization:

```bash
ros2 launch arm_description display_two_link.launch.py
```

Use `joint_state_publisher_gui` to adjust joint angles manually and verify URDF correctness. When the IK node is publishing joint states, RViz should reflect the computed configuration.

---

## Suggested Extensions

- Gazebo simulation and physics-based validation.
- Trajectory planning and execution (MoveIt 2).
- Joint controllers (ros2_control).
- Camera-based object tracking for closed-loop targeting.
- Reinforcement learning methods for control policy learning.

---

## Author
Panagiota Grosdouli

