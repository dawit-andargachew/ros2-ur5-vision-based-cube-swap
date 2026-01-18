# GROUP 6 – Assignment 2

This repository contains the code for **Assignment 2** of the *Intelligent Robotics* course.  

### Contributors
- **Dawit Andargachew Asmare**
- **Tharen Emmanuel Candi**
- **Giuseppe Castellana**

## Building

Navigate to your ROS2 workspace and build the package:

```bash
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

**Terminal 2** - Launch complete system (cube and color detector, action servers, orchestrator):
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

You can test the action servers individually using the provided launch files:

**Terminal 1** - Launch simulation and MoveIt:
```bash
ros2 launch ir_launch assignment_2.launch.py
```

**Terminal 2** - Launch all servers and services (AprilTag, detector, pick/place servers):
```bash
ros2 launch group_6_assignment_2 basic_a2_package.launch.py
```

**Terminal 3** - Test pick action:
```bash
ros2 launch group_6_assignment_2 pick_action_client.launch.py
```

**Terminal 4** - Test place action:
```bash
ros2 launch group_6_assignment_2 place_action_client.launch.py
```

---


### Testing Individual Actions

You can also send action goals directly via command line:

**Terminals 1-2**: Launch simulation and basic package as above

**Terminal 3** - Test picking cube 1:
```bash
ros2 action send_goal /pick group_6_assignment_2/action/Pick "{cube_id: 1}" --feedback
```

**Terminal 4** - Test placing cube (assumes cube 1 is already attached):
```bash
ros2 action send_goal /place group_6_assignment_2/action/Place "{grabbed_cube_id: 1, target_table_id: 2}" --feedback
```

---

## Configuration

### Pick Action Parameters

Edit [config/pick_action_server.yaml](config/pick_action_server.yaml) to tune grasp parameters:
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
            ↑
        CubeDetector (Service)

```

---

## BehaviorTree Logic

The swap logic is defined in [behaviour_trees/swap_logic.xml](behaviour_trees/swap_logic.xml):

1. Retry picking cube 1 and placing on table 2 (up to max_retries)
2. If successful, retry picking cube 10 and placing on table 1 (up to max_retries)
3. If either swap fails after retries, the entire operation fails
