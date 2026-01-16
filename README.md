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