# Two-Link Robotic Arm (ROS 2)

This project implements a complete ROS 2 workspace for a **2-DOF robotic arm**, including inverse kinematics, URDF modeling, visualization in RViz, and the foundation for future trajectory control and simulation.

It is designed as both an educational and experimental project for robotics, ROS 2, and kinematics.

---

## 🚀 Features

### ✔ Inverse Kinematics Node (Python)
Located in `arm_kinematics`, the node:
- Solves IK for a 2-link planar arm
- Publishes joint states
- Receives target (x, y) points as ROS 2 messages
- Runs on a 1-second timer for easy testing

Run with:

```bash
ros2 run arm_kinematics two_link_ik
✔ URDF Model of a 2-Link Arm

Located in arm_description/urdf/two_link.urdf.

Includes:

Base link

Link1 (0.3 m)

Link2 (0.2 m)

Two revolute joints

✔ RViz Visualization

A launch file is provided to visualize the robot with sliders:

ros2 launch arm_description display_two_link.launch.py


This opens:

RViz

joint_state_publisher_gui

robot_state_publisher

📂 Project Structure
two_link_arm_ros2/
│
├── arm_kinematics/         # ROS 2 Python package for IK
│   ├── two_link_ik.py
│   ├── setup.py
│   └── package.xml
│
├── arm_description/        # URDF, visualization, RViz
│   ├── urdf/two_link.urdf
│   ├── launch/display_two_link.launch.py
│   └── package.xml
│
├── build/ (ignored)
├── install/ (ignored)
├── log/ (ignored)
└── README.md

📦 Installation

Clone the repository:

git clone https://github.com/Lily-Evan/two_link_arm_ros2.git
cd two_link_arm_ros2


Build the workspace:

colcon build
source install/setup.bash

🧠 IK Node Usage

Run the inverse kinematics node:

ros2 run arm_kinematics two_link_ik


Send a custom target:

ros2 topic pub /target_point geometry_msgs/msg/Point "{x: 0.2, y: 0.1, z: 0.0}"

👀 Visualization

Launch the robot model in RViz:

ros2 launch arm_description display_two_link.launch.py


Use the GUI sliders to control joint angles.

🔮 Future Extensions

Integration with Gazebo simulation

Trajectory planning (MoveIt 2)

Joint controllers

Camera-based object tracking

Reinforcement Learning (RL) for control

👤 Author

Panagiota Grosdouli
