from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
def generate_launch_description():
    
    simulation = ExecuteProcess(
        cmd=['ros2', 'launch', 'ir_launch', 'assignment_1.launch.py', 'autostart:=True'],
        output='log',       # Sends stdout/stderr to ROS 2 log files
        emulate_tty=False   # Prevents output from appearing in terminal
    )


    # 1. Include the Simulation Launch File from ir_launch
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ir_launch"),
                "launch",
                "assignment_2.launch.py"
            ])
        )
    )


    # 2. Define the Pick Node
    pick_node = Node(
        package="group_6_assignment_2",
        executable="pick",
        name="pick_node",
        output="screen"
    )

    # 3. Delay the pick node by 10 seconds to allow MoveIt/Gazebo to fully load
    delayed_pick = TimerAction(
        period=12.0,
        actions=[pick_node]
    )

    return LaunchDescription([
        simulation,
        delayed_pick
    ])
