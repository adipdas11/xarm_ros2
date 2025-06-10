# disassembly\_v3

A modular ROS 2 package for automated disassembly using a YOLO vision model, Cartesian velocity control, effort-based contact detection, and screwdriver actuation. This README explains the overall architecture, control flow, individual component responsibilities, and how to test/run each part.

---

## Table of Contents

1. [Overview](#overview)
2. [Package Structure](#package-structure)
3. [Runtime Flow](#runtime-flow)
4. [Disassembly Sequence Flow](#disassembly-sequence-flow)
5. [Node & Service/Action Descriptions](#node--serviceaction-descriptions)
6. [Running & Testing Individual Components](#running--testing-individual-components)
7. [End-to-End Launch](#end-to-end-launch)
8. [Troubleshooting](#troubleshooting)

---

## Overview

1. **Vision → Graph**: `vision_graph_node.py` runs YOLOv11 on synced color+depth images, back-projects centroids into 3D, builds a proximity graph of detected parts, and exposes it via a topic and service.
2. **Sequence Planning**: `sequence_planner_node.py` subscribes to the part graph, filters and sorts `screw_*` parts by priority, publishes the disassembly order, and offers it via a service.
3. **Pose Transform**: `pose_transform_service.py` provides a TF2-based service to transform 3D poses between frames (e.g., camera → robot base).
4. **Motion Primitives (Actions)**

   * **`MoveToPose`**: PID-based Cartesian velocity servo to move the TCP to any `PoseStamped`.
   * **`ApproachUntilContact`**: Hover above a target then descend until joint effort crosses a threshold (detects contact).
   * **`UnscrewScrew`**: Spins the tool motor, monitors effort spikes, and retracts if needed until the screw is removed.
5. **Orchestration**: `disassembly_orchestrator_node.py` orchestrates the full pipeline by calling services and actions in sequence for each screw in the generated plan.

---

## Package Structure

```
disassembly_v3/
├── action/                    # Action definitions
│   ├── ApproachUntilContact.action
│   ├── MoveToPose.action
│   └── UnscrewScrew.action
├── srv/                       # Service definitions
│   ├── GetPartGraph.srv
│   ├── GetSequence.srv
│   └── TransformPose.srv
├── scripts/                   # Python executables
│   ├── vision_graph_node.py
│   ├── sequence_planner_node.py
│   ├── pose_transform_service.py
│   ├── velocity_servo_action_server.py
│   ├── contact_approach_action_server.py
│   ├── unscrew_action_server.py
│   └── disassembly_orchestrator_node.py
├── launch/                    # Full-system launch
│   └── orchestrator_launch.py
├── CMakeLists.txt             # Build definitions
└── package.xml                # Dependencies
```

---

## Runtime Flow

1. **Startup**: Launch nodes (individually or via `orchestrator_launch.py`).
2. **Vision Graph**: `vision_graph_node` publishes `/part_graph` and serves `GetPartGraph`.
3. **Planning**: `sequence_planner_node` reads `/part_graph`, publishes `/disassembly_sequence`, and serves `GetSequence`.
4. **Orchestrator**: `disassembly_orchestrator_node` loops:

   * Calls `GetSequence` → for each screw:

     * Calls `GetPartGraph` to get camera-frame coordinates.
     * Calls `TransformPose` to convert to base frame.
     * Sends `ApproachUntilContact` action.
     * Sends `UnscrewScrew` action.
     * Sends `MoveToPose` action to return home.
   * Repeats until the sequence is empty.

---

## Disassembly Sequence Flow

The disassembly process is orchestrated across multiple nodes:

1. **VisionGraphNode** processes camera data, detects parts, builds a 3D graph, and makes it available.
2. **SequencePlannerNode** filters and sorts screw parts, producing an ordered removal list.
3. **DisassemblyOrchestratorNode** drives the operation:

   * Retrieves the next screw ID via `GetSequence`.
   * Obtains its 3D position via `GetPartGraph`.
   * Transforms that pose into the robot base frame via `TransformPose`.
   * Executes the `ApproachUntilContact` action to approach and detect contact.
   * Executes the `UnscrewScrew` action to remove the screw.
   * Executes the `MoveToPose` action to return to the home pose.
   * Loops to the next screw until complete.

---

## Node & Service/Action Descriptions

Detailed at the top of each script for clarity.

---

## Running & Testing Individual Components

Steps for each node, service, and action using `ros2 run`, `ros2 service call`, `ros2 topic pub`, and `ros2 action send_goal`.
(Refer to earlier section for exact commands.)

---

## Manual Node Execution Sequence

If you prefer to start each component individually rather than using the launch file, run them in separate terminals in this order:

1. **Vision Graph Node** (choose `sim` or `real` mode via parameter)

   ```bash
   # For simulation mode (default):
   ros2 run disassembly_v3 vision_graph_node --ros-args -p mode:=sim

   # For real hardware mode:
   ros2 run disassembly_v3 vision_graph_node --ros-args -p mode:=real
   ```
   
2. **Sequence Planner Node**

   ```bash
   ros2 run disassembly_v3 sequence_planner_node
   ```
3. **Pose Transform Service**

   ```bash
   ros2 run disassembly_v3 pose_transform_service
   ```
4. **MoveToPose Action Server**

   ```bash
   ros2 run disassembly_v3 velocity_servo_action_server
   ```
5. **ApproachUntilContact Action Server**

   ```bash
   ros2 run disassembly_v3 contact_approach_action_server
   ```
6. **UnscrewScrew Action Server**

   ```bash
   ros2 run disassembly_v3 unscrew_action_server
   ```
7. **Orchestrator Node**

   ```bash
   ros2 run disassembly_v3 disassembly_orchestrator_node
   ```

---

## End-to-End Launch

```bash
ros2 launch disassembly_v3 orchestrator_launch.py
```

---
