# xArm ROS2 Workspace Setup and Usage

This repository provides instructions to set up a ROS2 workspace for the xArm manipulator, including optional camera calibration, MoveIt2 tutorials, and the xArm Python SDK. It also documents useful ROS2 services to control the robot, gripper, and linear motor.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Workspace Setup](#workspace-setup)
   - [Create Workspace](#create-workspace)
   - [Clone Repositories](#clone-repositories)
3. [Building the Workspace](#building-the-workspace)
4. [Optional: Camera Calibration](#optional-camera-calibration)
5. [Optional: Python SDK for xArm](#optional-python-sdk-for-xarm)
6. [Useful Services](#useful-services)
   - [Pose Planning](#pose-planning)
   - [Execute Plan](#execute-plan)
   - [Gripper Joint Planning](#gripper-joint-planning)
   - [Gripper Execution](#gripper-execution)
   - [Linear Motor Control](#linear-motor-control)

---

## Prerequisites

- ROS2 (e.g., Humble Hawksbill) installed and sourced.
- `git`, `colcon`, `vcs`, and ROS2 dependencies (`rosdep`).
- Appropriate permissions to install packages (`sudo` access).

---

## Workspace Setup

### Create Workspace

```bash
# Skip this step if you already have a target workspace
cd ~
mkdir -p dev_ws/src
cd ~/dev_ws/src
```

### Clone Repositories

```bash
git clone https://github.com/adipdas11/xarm_ros2.git
```

---

## Building the Workspace

Install dependencies and build:

```bash
cd ~/dev_ws
sudo apt update
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --mixin release
```

---

## Optional: Camera Calibration

If you need camera calibration and MoveIt2 tutorials, run:

```bash
cd ~/dev_ws/src
# Clone MoveIt2 tutorials for Humble
git clone https://github.com/ros-planning/moveit2_tutorials --branch humble
# Clone calibration package
git clone git@github.com:adipdas11/moveit2_calibration.git

# Import repositories
vcs import < moveit2_tutorials/moveit2_tutorials.repos

# Install dependencies and rebuild
cd ~/dev_ws
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --mixin release
```

---

## Optional: Python SDK for xArm

To control the xArm via Python:

```bash
cd ~/dev_ws/src
git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
cd xArm-Python-SDK
python3 setup.py install
pip3 install xarm-python-sdk
```

---

## Useful Services

Below are the primary ROS2 services for controlling the xArm manipulator and its peripherals.

### Pose Planning

- **Service:** `/xarm_pose_plan`
- **Type:** `xarm_msgs/srv/PlanPose`
- **Request Format:**
  ```ros
  geometry_msgs/Pose target
  ---
  bool success
  ```

- **Example:**
  ```bash
  ros2 service call /xarm_pose_plan xarm_msgs/srv/PlanPose "{target:
       {position:   {x: 0.3,   y: -0.1,  z: 0.2},
        orientation:{x: 1.0,   y:  0.0,  z: 0.0,  w: 0.0}
       }
     }"
  ```

### Execute Plan

- **Service:** `/xarm_exec_plan`
- **Type:** `xarm_msgs/srv/PlanExec`
- **Request Format:**
  ```ros
  bool wait
  ---
  bool success
  ```

- **Example:**
  ```bash
  ros2 service call /xarm_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"
  ```

### Gripper Joint Planning

- **Service:** `/xarm_gripper_joint_plan`
- **Type:** `xarm_msgs/srv/PlanJoint`
- **Request Format:**
  ```ros
  float64[] target
  ---
  bool success
  ```

- **Examples:**
  - **Close Gripper**
    ```bash
    ros2 service call /xarm_gripper_joint_plan xarm_msgs/srv/PlanJoint "{ target: [0.85, 0.85, 0.85, 0.85, 0.85, 0.85] }"
    ```

  - **Open Gripper**
    ```bash
    ros2 service call /xarm_gripper_joint_plan xarm_msgs/srv/PlanJoint "{ target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] }"
    ```

### Gripper Execution

- **Service:** `/xarm_gripper_exec_plan`
- **Type:** `xarm_msgs/srv/PlanExec`
- **Request Format:**
  ```ros
  bool wait
  ---
  bool success
  ```

- **Example:**
  ```bash
  ros2 service call /xarm_gripper_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"
  ```

### Linear Motor Control

- **Service:** `/move_linear_motor`
- **Type:** `ufactory_linear_motor_description/srv/MoveLinearMotor`
- **Request Format:**
  ```ros
  float64 target_position_m
  ---
  bool success
  string message
  ```

- **Example (move to 0.7 m):**
  ```bash
  ros2 service call /move_linear_motor ufactory_linear_motor_description/srv/MoveLinearMotor "{target_position_m: 0.7}"
  ```

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

