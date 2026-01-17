# GROUP 6 – Assignment 2

This repository contains the code for **Assignment 2** of the *Intelligent Robotics* course.  

### Contributors
- **Dawit Andargachew Asmare**
- **Tharen Emmanuel Candi**
- **Giuseppe Castellana**

---

## Overview

This package implements a complete pick-and-place system for swapping cubes between tables using a UR5 robotic arm with MoveIt2. The system consists of:

- **CubeDetector**: Detects cube poses using AprilTag markers
- **PickActionServer**: ROS2 action server for picking cubes
- **PlaceActionServer**: ROS2 action server for placing cubes
- **Orchestrator**: BehaviorTree-based high-level coordinator that swaps cubes between tables

---

## Prerequisites

Install the required dependencies:

```bash
sudo apt install ros-${ROS_DISTRO}-behaviortree-cpp
```

**Note**: This package includes embedded BehaviorTree.ROS2 action client code (Apache 2.0 licensed) from the [behaviortree_ros2](https://github.com/BehaviorTree/BehaviorTree.ROS2) project by Davide Faconti. This eliminates the need for external ROS2 package dependencies while maintaining full orchestrator functionality.

---

## Building

Navigate to your ROS2 workspace and build the package:

```bash
cd ~/MASTERS/robotics/labs/ws_6_assignments
colcon build --packages-select group_6_assignment_2
source install/setup.bash
```

---

## Running the System

### Complete System with Orchestrator (Recommended)

This launches all components including the BehaviorTree orchestrator that automatically swaps cubes between tables.

**Terminal 1** - Launch simulation and MoveIt:
```bash
ros2 launch ir_launch assignment_2.launch.py
```

**Terminal 2** - Launch complete system (cube detector, action servers, orchestrator):
```bash
ros2 launch group_6_assignment_2 orchestrator_full.launch.py use_sim_time:=true max_retries:=3
```

The orchestrator will:
1. Wait 5 seconds for all services to initialize
2. Pick cube 1 from table 1 and place it on table 2
3. Pick cube 10 from table 2 and place it on table 1
4. Retry each operation up to `max_retries` times on failure

---

### Testing Without Orchestrator

If `behaviortree_ros2` is unavailable, you can test the action servers individually:

**Terminal 1** - Launch simulation and MoveIt:
```bash
ros2 launch ir_launch assignment_2.launch.py
```

**Terminal 2** - Launch action servers and detector:
```bash
ros Testing Individual Actions

For validation or debugging of specific operations:

#### Test Pick Action Only

**Terminal 1-2**: Launch simulation and basic package

**Terminal 3** - Test picking cube 1:
```bash
ros2 action send_goal /pick group_6_assignment_2/action/Pick "{cube_id: 1}" --feedback
```

#### Test Place Action Only

**Terminal 1-2**: Launch simulation and basic package

**Terminal 3

### Advanced: Manual Component Launch

For debugging or testing individual components:

**Terminal 1** - Launch simulation and MoveIt:
```bash
ros2 launch ir_launch assignment_2.launch.py
```

**Terminal 2** - Cube detector:
```bash
ros2 run group_6_assignment_2 cube_detector
```

**Terminal 3** - Pick action server:
```bash
ros2 run group_6_assignment_2 pick_action_server --ros-args -p use_sim_time:=true --params-file install/group_6_assignment_2/share/group_6_assignment_2/config/pick_params.yaml
```

**Terminal 4** - Place action server:
```bash
ros2 run group_6_assignment_2 place_action_server --ros-args -p use_sim_time:=true
```

**Terminal 5** - Orchestrator (optional):
```bash
ros2 run group_6_assignment_2 orchestrator --ros-args -p use_sim_time:=true -p max_retries:=3
```

---

### Testing Individual Actions

#### Test Pick Action Only

**Terminal 1-3**: Same as above (simulation, cube detector, pick server)

**Terminal 4** - Test picking cube 1:
```bash
ros2 action send_goal /pick group_6_assignment_2/action/Pick "{cube_id: 1}" --feedback
```

#### Test Place Action Only

**Terminal 1-4**: Launch simulation, detector, and both action servers

**Terminal 5** - Test placing cube (assumes cube 1 is already attached):
```bash
ros2 action send_goal /place group_6_assignment_2/action/Place "{grabbed_cube_id: 1, target_table_id: 2}" --feedback
```

---

## Configuration

### Pick Action Parameters

Edit [config/pick_params.yaml](config/pick_params.yaml) to tune grasp parameters:
- Grasp orientation (roll, pitch, yaw)
- Grasp offsets from cube center
- Pre-grasp approach distance
- Lift height after grasping

### Orchestrator Parameters

- `max_retries`: Number of retry attempts per pick/place operation (default: 3)
- `use_sim_time`: Set to true for simulation (default: true)

---

## Architecture

```
Orchestrator (BehaviorTree)
    ↓
    ├── PickAction (Action Client)
    │       ↓
    │   PickActionServer → MoveIt → UR5 Robot
    │       ↑
    │   CubeDetector (Service)
    │
    └── PlaceAction (Action Client)
            ↓
        PlaceActionServer → MoveIt → UR5 Robot
```

### Action Interfaces

**Pick Action**
- Goal: `cube_id` (uint8)
- Result: `success` (bool)
- Feedback: `status` (string)

**Place Action**
- Goal: `grabbed_cube_id` (uint8), `target_table_id` (uint8)
- Result: `success` (bool)
- Feedback: `status` (string)

---

## BehaviorTree Logic

The swap logic is defined in [behaviour_trees/swap_logic.xml](behaviour_trees/swap_logic.xml):

1. Retry picking cube 1 and placing on table 2 (up to max_retries)
2. If successful, retry picking cube 10 and placing on table 1 (up to max_retries)
3. If either swap fails after retries, the entire operation fails

---

## Troubleshooting

**Issue**: Compilation error `behaviortree_cpp not found`
```bash
sudo apt install ros-${ROS_DISTRO}-behaviortree-cpp
```

**Issue**: Orchestrator can't find XML file
- Ensure you've run `colcon build` and sourced `install/setup.bash`
- Check that `behaviour_trees/swap_logic.xml` exists in the package
- Verify installation: `ls install/group_6_assignment_2/share/group_6_assignment_2/behaviour_trees/`

**Issue**: Action servers not reachable
- Check all servers are running: `ros2 action list` (should show `/pick` and `/place`)
- Verify nodes: `ros2 node list` (should show `pick_action_server`, `place_action_server`)
- Increase orchestrator startup delay in [orchestrator_full.launch.py](launch/orchestrator_full.launch.py) if needed

**Issue**: Pick/place actions fail
- Check action server logs for planning failures
- Verify MoveIt is properly loaded: `ros2 topic list | grep move_group`
- Tune motion parameters in [config/pick_params.yaml](config/pick_params.yaml)
- Check workspace bounds and collision objects

**Issue**: Cubes not detected
- Ensure AprilTag IDs 1 and 10 are visible to the camera
- Check cube detector is running: `ros2 service list | grep get_cube_poses`
- Verify camera topic: `ros2 topic echo /camera/rgb/image_raw --once`
- Check AprilTag detection: `ros2 topic echo /apriltag/detections`

**Issue**: Simulation not starting
- Ensure Gazebo is properly installed: `which gz`
- Check for port conflicts: `killall -9 gz gzclient gzserver`
- Verify ROS2 workspace is sourced: `echo $ROS_DISTRO

---

## Development

### Adding New BehaviorTree Logic

1. Create a new XML file in `behaviour_trees/`
2. Modify [Orchestrator.cpp](src/Orchestrator.cpp) to load your XML file
3. Rebuild the package

### Modifying Action Nodes

Edit [orchestrator_action_nodes.hpp](include/group_6_assignment_2/orchestrator_action_nodes.hpp) to customize BehaviorTree node behavior.

---
three terminals to test pick

ros2 launch ir_launch assignment_2.launch.py
ros2 launch group_6_assignment_2 basic_a2_package.launch.py
ros2 launch group_6_assignment_2 pick_action_client.launch.py
ros2 launch group_6_assignment_2 place_action_client.launch.py
```


### How to run cube color detector (need 3 terminals)

```bash
    ros2 launch ir_launch assignment_2.launch.py
    ros2 run group_6_assignment_2 color_detector
    ros2 service call /get_cube_colors group_6_assignment_2/srv/GetCubeColors {}
```
