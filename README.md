# GROUP 6 – Assignment 2

This repository contains the code for **Assignment 2** of the *Intelligent Robotics* course.  
### Contributors
- **Dawit Andargachew Asmare**
- **Tharen Emmanuel Candi**
- **Giuseppe Castellana**.

---

## How to Run

After cloning the repository, navigate to your ROS2 workspace and run:

```bash
colcon build --packages-select group_6_assignment_2
source install/setup.bash


ros2 launch ir_launch assignment_2.launch.py
ros2 run group_6_assignment_2 pick
