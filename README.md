# Six-DOF Robot Arm – ROS 2

A six-degree-of-freedom robotic arm project developed using ROS 2 Jazzy. The project focuses on robot modelling, kinematics, inverse kinematics, trajectory generation and visualization, with Gazebo simulation and pick-and-place functionality planned for future development.

## Project Overview

The project uses a generic six-DOF robot arm model as a foundation for developing and testing robotic manipulation algorithms. The architecture is designed to support future integration with the Yaskawa GP7 industrial robot model.

## Implemented Features

- **Robot Modelling:** Six-DOF robotic arm described using URDF and Xacro.
- **Forward Kinematics:** End-effector pose calculation using homogeneous transformation matrices.
- **Denavit–Hartenberg (DH) Parameters:** Forward kinematics implementation using standard DH convention.
- **Jacobian:** Calculation of the robot's position Jacobian.
- **Inverse Kinematics:** Position-based inverse kinematics using the damped least squares method.
- **Trajectory Generation:** Smooth joint-space motion using quintic time scaling.
- **ROS 2 Integration:** Joint-state publishing and robot state visualization.
- **RViz Visualization:** Real-time robot model and TF visualization.

## Technology Stack

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- NumPy
- URDF / Xacro
- RViz2
- TF2

## Repository Structure

```text
robot_arm/
├── scripts/
│   ├── forward_kinematics.py
│   ├── dh_forward_kinematics.py
│   ├── jacobian.py
│   ├── ik_position.py
│   └── publish_ik_solution.py
│
├── src/
│   └── robot_arm_description/
│       ├── launch/
│       │   └── display.launch.py
│       ├── urdf/
│       │   └── generic/
│       │       └── robot_arm.urdf.xacro
│       ├── meshes/
│       ├── config/
│       ├── rviz/
│       ├── CMakeLists.txt
│       └── package.xml
│
├── .gitignore
└── README.md
```

## Setup

### 1. Prerequisites

Install ROS 2 Jazzy and the required packages.

```bash
sudo apt update

sudo apt install \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-ros \
    python3-numpy
```

### 2. Clone the Repository

```bash
git clone <YOUR_REPOSITORY_URL>
cd robot_arm
```

### 3. Build the ROS 2 Workspace

```bash
source /opt/ros/jazzy/setup.bash

colcon build

source install/setup.bash
```

### 4. Launch the Robot Model

```bash
ros2 launch robot_arm_description display.launch.py
```

This launches the robot state publisher and RViz2 for robot visualization.

## Running the Kinematics Scripts

Run the scripts from the workspace root directory.

**Forward Kinematics:**

```bash
python3 scripts/forward_kinematics.py
```

**DH Forward Kinematics:**

```bash
python3 scripts/dh_forward_kinematics.py
```

**Jacobian Calculation:**

```bash
python3 scripts/jacobian.py
```

**Position Inverse Kinematics:**

```bash
python3 scripts/ik_position.py
```

## Executing a Smooth Joint-Space Trajectory

First, launch the robot model in one terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch robot_arm_description display.launch.py
```

Then, in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/publish_ik_solution.py
```

The inverse kinematics solution is published to `/joint_states`, allowing the robot model to move smoothly in RViz.

## Planned Development

- Gazebo Harmonic simulation.
- Robot dynamics, collision and inertial modelling.
- ROS 2 Control integration.
- Joint Trajectory Controller.
- MoveIt 2 motion planning.
- Integration of the Yaskawa GP7 robot model.
- Gripper modelling and control.
- Pick-and-place task implementation.

## Project Status

**Current stage:** Generic six-DOF robot modelling, kinematics, inverse kinematics, trajectory generation and ROS 2 visualization.

This project is under active development.
