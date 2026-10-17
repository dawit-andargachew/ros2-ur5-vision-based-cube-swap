<div align="center">

# 🤖 UR5 Cube Swap

**A simulated UR5 arm that finds two cubes with AprilTags and swaps them between two tables, orchestrated by a BehaviorTree with automatic retries.**

![ROS 2](https://img.shields.io/badge/ROS%202-Humble%2B-22314E?logo=ros)
![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)
![MoveIt](https://img.shields.io/badge/MoveIt-2-orange)
![BehaviorTree.CPP](https://img.shields.io/badge/BehaviorTree.CPP-v4-green)
![License](https://img.shields.io/badge/License-Apache%202.0-blue)

<a href="https://drive.google.com/file/d/1EAvlPeIlvL5cCZah2a6z21FSKIIr2TdN/view?usp=sharing">
  <img src="docs/images/demo-video-thumbnail.jpg" alt="Watch the demo video" width="720">
</a>

*Click to watch the demo video (opens Google Drive)*

</div>

---

## Overview

This project was built for *Intelligent Robotics* (MSc), Assignment 2. A UR5 arm with a gripper stands between two tables. Each table holds one cube with an **AprilTag** on top. The robot has to **swap the two cubes**, then go back to its home pose and report the cubes' colors.

<p align="center">
  <img src="docs/images/ur5-cube-swap-scene.png" alt="Simulation scene: a UR5 arm between two tables, each with a tagged cube" width="640">
  <br>
  <em>The simulated scene: the UR5 sits between two tables. One table holds a red cube and the other a blue cube, and each cube has an AprilTag on its top face.</em>
</p>

**What the scene shows**

| Element | Role |
|---|---|
| UR5 arm with gripper | Picks up and places the cubes |
| Two tables | Source and destination for each cube (table 1 and table 2) |
| Red and blue cubes | The objects to swap; the color detector reports these |
| AprilTag on each cube | Lets the vision node compute the cube's 3D pose |

---

## Features

- **Vision-based grasping:** cube poses come from AprilTag detections, so cubes don't need to be at hard-coded positions.
- **Color detection:** reads each cube's color from the camera image.
- **Modular design:** pick, place and home are independent ROS 2 action servers that you can test one at a time.
- **Robust execution:** a BehaviorTree runs the task and retries any failed step.
- **Configurable:** grasp offsets, poses and retry counts live in YAML files.

---

## How It Works

### System at a glance

```mermaid
flowchart LR
    subgraph Brain["🧠 Decision making"]
        O["Orchestrator<br/>(BehaviorTree)"]
    end

    subgraph Skills["🦾 Robot skills (ROS 2 actions)"]
        P["Pick"]
        PL["Place"]
        H["Home"]
    end

    subgraph Perception["👁 Perception (ROS 2 services)"]
        CD["Cube detector<br/>poses from AprilTags"]
        CC["Color detector<br/>cube colors"]
    end

    subgraph Sim["🌍 Simulation"]
        M["MoveIt planner"]
        R["UR5 + gripper<br/>+ camera"]
    end

    O -->|"goal"| P
    O -->|"goal"| PL
    O -->|"goal"| H
    O -->|"request"| CC
    P -->|"where is the cube?"| CD
    PL -->|"where is the cube?"| CD
    P --> M
    PL --> M
    H --> M
    M --> R
    R -.->|"camera image"| CD
    R -.->|"camera image"| CC

    classDef brain fill:#fde68a,stroke:#b45309,color:#111;
    classDef skill fill:#bfdbfe,stroke:#1d4ed8,color:#111;
    classDef perc fill:#bbf7d0,stroke:#15803d,color:#111;
    classDef sim fill:#e5e7eb,stroke:#4b5563,color:#111;
    class O brain;
    class P,PL,H skill;
    class CD,CC perc;
    class M,R sim;
```

In short: the **orchestrator** decides *what* to do, the **action servers** know *how* to do it, **perception** tells them *where* things are, and **MoveIt** turns targets into safe arm motions.

### The task, step by step

```mermaid
flowchart TD
    A([Start]) --> B["1. Detect cube colors"]
    B --> C["2. Pick cube 1"]
    C --> D["3. Place on table 2"]
    D --> E["4. Pick cube 10"]
    E --> F["5. Place on table 1"]
    F --> G["6. Return home"]
    G --> H["7. Show detected colors"]
    H --> I([Done])

    C -. "fails" .-> R{{"Retry<br/>(up to max_retries)"}}
    D -. "fails" .-> R
    E -. "fails" .-> R
    F -. "fails" .-> R
    R -. "still failing" .-> X([Task fails])
```

Each of the pick, place and home steps is wrapped in a retry decorator. If a step still fails after `max_retries` attempts, the whole run fails. The tree itself is defined in [`behaviour_trees/swap_logic.xml`](behaviour_trees/swap_logic.xml).

### Building blocks

| Component | What it does |
|---|---|
| **Orchestrator** | Runs the BehaviorTree and calls the other components in order |
| **Cube detector** | Service `get_cube_poses`: turns AprilTag detections into cube poses |
| **Color detector** | Service `get_cube_colors`: returns each cube's color from the camera image |
| **Pick server** | Action `pick`: approaches, grasps and lifts the requested cube |
| **Place server** | Action `place`: puts the held cube on the target table |
| **Home server** | Action `home`: moves the arm to its safe home pose |
| **Test clients** | Small clients to trigger each action by hand |

### Interfaces

| Type | Name | Input | Output |
|---|---|---|---|
| Action | `Pick` | `cube_id` | `success` |
| Action | `Place` | `grabbed_cube_id`, `target_table_id` | `success` |
| Action | `Home` | *(none)* | `success`, `message` |
| Service | `GetCubePoses` | `target_ids[]` | `success`, `message`, `detected_ids[]`, `poses[]` |
| Service | `GetCubeColors` | *(none)* | `success`, `message` (e.g. `Cube 1: RED \| Cube 10: BLUE`) |

---

## Getting Started

### Prerequisites

- ROS 2 with `colcon`, **MoveIt 2** and **BehaviorTree.CPP v4**
- The course workspace packages **`ir_launch`** (simulation) and **`ir_movit_config`** (MoveIt configuration)
- Other dependencies: AprilTag (`apriltag_msgs`), OpenCV / `cv_bridge`, `tf2`, `image_geometry`

### Build

From your ROS 2 workspace:

```bash
colcon build --packages-select group_6_assignment_2
source install/setup.bash
```

> Source `install/setup.bash` in **every new terminal**.

### Run the full swap

**Terminal 1:** simulation and MoveIt
```bash
ros2 launch ir_launch assignment_2.launch.py
```

**Terminal 2:** detectors, action servers and orchestrator
```bash
ros2 launch group_6_assignment_2 orchestrator_full.launch.py use_sim_time:=true max_retries:=5
```

| Argument | Default | Meaning |
|---|---|---|
| `use_sim_time` | `true` | Use the simulation clock |
| `max_retries` | `5` | Attempts per pick / place / home step |

The launch file starts the nodes one after another with short delays, and the orchestrator begins about 8 seconds in, once the servers are ready. Watch its log for action feedback and retries.

---

## Testing Pieces Individually

Handy for debugging one step without running the whole task.

**Terminal 1:** simulation (as above)
```bash
ros2 launch ir_launch assignment_2.launch.py
```

**Terminal 2:** detectors and action servers, without the orchestrator
```bash
ros2 launch group_6_assignment_2 basic_a2_package.launch.py
```

**Terminal 3:** trigger an action with a client launch file…
```bash
ros2 launch group_6_assignment_2 pick_action_client.launch.py
ros2 launch group_6_assignment_2 place_action_client.launch.py
ros2 launch group_6_assignment_2 home_action_client.launch.py
```

…or send a goal directly:
```bash
# Pick cube 1
ros2 action send_goal /pick group_6_assignment_2/action/Pick "{cube_id: 1}" --feedback

# Place the held cube on table 2 (cube 1 must already be grasped)
ros2 action send_goal /place group_6_assignment_2/action/Place "{grabbed_cube_id: 1, target_table_id: 2}" --feedback

# Go to the home pose
ros2 action send_goal /home group_6_assignment_2/action/Home "{}" --feedback
```

---

## Configuration

| File | What it controls |
|---|---|
| [`config/orchestrator_config.yaml`](config/orchestrator_config.yaml) | Orchestrator settings such as `max_retries` |
| [`config/planning_configuration.yaml`](config/planning_configuration.yaml) | Home and active joint poses, table position, cube and tag sizes, grasp orientation and offsets, approach and lift distances |
| [`config/custom_tags_36h11.yaml`](config/custom_tags_36h11.yaml) | AprilTag (`36h11`) detector settings |

| Orchestrator parameter | Default | Meaning |
|---|---|---|
| `max_retries` | `3` in code, `5` via the launch file | Attempts per step |
| `enable_bt_logging` | `true` | Print detailed BehaviorTree tick and retry logs |
| `use_sim_time` | `true` | Use the simulation clock |

To tune grasping, edit `config/planning_configuration.yaml`, then rebuild so the installed copy is updated.

---

## Troubleshooting

| Symptom | What to try |
|---|---|
| Servers not found, nothing happens | Start the simulation (Terminal 1) first, and source `install/setup.bash` in each terminal |
| Not sure what is running | `ros2 action list` (expect `/pick`, `/place`, `/home`) and `ros2 service list` (expect `/get_cube_poses`, `/get_cube_colors`) |
| A step keeps failing | Raise `max_retries`, or run that step alone and read the feedback |
| `Place` fails when run alone | It needs a cube already grasped, so run `Pick` first |

---

## Repository Layout

```
├── action/            Pick, Place, Home action definitions
├── srv/               GetCubePoses, GetCubeColors service definitions
├── behaviour_trees/   swap_logic.xml (the task)
├── config/            YAML parameters
├── launch/            Launch files (full system, basic servers, test clients)
├── src/, include/     C++ node implementations and headers
└── docs/images/       README images
```

---

## Team Members

**[Dawit Andargachew Asmare](https://github.com/dawit-andargachew) · [Tharen Emmanuel Candi](https://github.com/tharencandi) · [Giuseppe Castellana](https://github.com/g-castel15)**

## License

Released under the [Apache License 2.0](LICENSE).
