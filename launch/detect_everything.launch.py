from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. Launch the Color Detector (The "Logic")
    color_detector_node = Node(
        package='group_6_assignment_2',
        executable='color_detector',
        name='color_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] 
    )
    
    # 2. Cube Detector Service (Optional, kept for completeness)
    cube_detector_node = Node(
        package='group_6_assignment_2',
        executable='cube_detector',
        name='cube_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        color_detector_node,
        cube_detector_node
    ])
