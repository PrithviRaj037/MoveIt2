# ROS 2 MoveIt2 Robotic Arm Simulation with Gazebo
## Overview
The goal of this project is to build a complete simulated robotic arm pipeline using ROS 2.

The robot is modeled using **URDF/Xacro**, controlled using **ros2_control**, planned using **MoveIt 2**, visualized in **RViz**, and simulated in **Gazebo**.

Current motion sequence:
1. Open gripper
2. Move end-effector to pick pose
3. Close parallel gripper
4. Lift the end-effector
5. Move to place location
6. Open gripper to release

## Demo
https://github.com/user-attachments/assets/52f3dec3-890c-4108-a1e8-1225c53d57d9

## System Architecture
```text
URDF/Xacro Robot Model
        ↓
robot_state_publisher
        ↓
ros2_control + Gazebo Hardware Interface
        ↓
Joint Trajectory Controllers
        ↓
MoveIt 2 Planning Pipeline
        ↓
C++ MoveGroupInterface Commander
        ↓
Gazebo Simulation + RViz Visualization
```
## Features
1. 6-axis robotic arm simulation
2. Parallel gripper integration
3. Modular URDF/Xacro robot description
4. MoveIt 2 planning groups for arm and gripper
5. C++ motion execution using MoveGroupInterface
6. Gazebo simulation environment
7. RViz visualization and motion planning
8. ros2_control controller setup
9. Arm trajectory controller
10. Gripper trajectory controller
11. Basic pick, close, lift, place, and release sequence
## Project Structure
```text
MoveIt2/
├── assets/
│   └── videos/
│       └── pick_place_demo.mp4
├── src/
│   ├── my_robot_bringup/
│   │   ├── config/
│   │   ├── launch/
│   │   └── worlds/
│   ├── my_robot_commander_cpp/
│   │   └── src/
│   │       └── test_moveit.cpp
│   ├── my_robot_description/
│   │   ├── urdf/
│   │   └── meshes/
│   └── my_robot_moveit_config/
│       └── config/
├── README.md
└── .gitignor
```

## Requirements
This project was developed with:

1. Ubuntu 24.04
2. ROS 2 Jazzy
3. Gazebo Harmonic
4. MoveIt 2
5. RViz2
6. ros2_control
7. C++
8. colcon build system

Required ROS 2 packages include:
```text
sudo apt install ros-jazzy-moveit
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-ros-gz
```
## Build Instructions
Clone the repository:
```text
git clone https://github.com/PrithviRaj037/MoveIt2.git
cd MoveIt2
```
Build the workspace:
```text
colcon build
source install/setup.bash
```
If you are already inside the workspace:
```text
cd ~/ros2_ws
colcon build
source install/setup.bash
```
## Run Instructions
```text
ros2 launch my_robot_bringup my_robot.launch.xml
```
In another terminal, source the workspace:
```text
cd ~/ros2_ws
source install/setup.bash
```
Run the C++ MoveIt commander node:
```text
ros2 run my_robot_commander_cpp test_moveit --ros-args -p use_sim_time:=true
```
Check active controllers:
```text
ros2 control list_controllers
```
Expected controllers:
```text
joint_state_broadcaster    active
arm_controller             active
gripper_controller         active
```
If the gripper controller is inactive, activate it:
```text
ros2 control switch_controllers --activate gripper_controller --strict
```
## Challenges Solved
During this project, I worked through several practical ROS 2 and robotics simulation issues:

1. Creating a custom robot model using URDF/Xacro
2. Connecting the robot model with MoveIt 2
3. Setting up planning groups for the arm and gripper
4. Configuring ros2_control controllers
5. Connecting MoveIt 2 execution with Gazebo simulation hardware
6. Debugging inactive controller issues
7. Solving gripper controller execution problems
8. Understanding joint limits for a parallel gripper
9. Debugging RViz, Gazebo, TF, and joint state synchronization
10. Handling simulation lag on a low-resource laptop

One important lesson was that MoveIt may successfully generate a motion plan, but execution can still fail if the required controller is inactive or incorrectly configured.
## Future Work
1. Add ArUco marker-based object localization
2. Add camera-based object detection
3. Add Docker support for reproducible setup



