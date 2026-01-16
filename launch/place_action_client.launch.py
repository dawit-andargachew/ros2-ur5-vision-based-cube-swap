from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    grabbed_cube_id = LaunchConfiguration("grabbed_cube_id")
    target_table_id = LaunchConfiguration("target_table_id")

    # Keep the exact same MoveIt config pattern as the example scaffold.
    moveit_config_pkg = "ir_movit_config"
    moveit_config = MoveItConfigsBuilder(
        "ir_gripper", package_name=moveit_config_pkg
    ).to_moveit_configs()

    place_action_client_node = Node(
        package="group_6_assignment_2",
        executable="place_action_client",
        name="place_action_client",
        output="screen",
        arguments=[grabbed_cube_id, target_table_id],
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "grabbed_cube_id",
                default_value="1",
                description="ID of the cube that is grabbed",
            ),
            DeclareLaunchArgument(
                "target_table_id",
                default_value="2",
                description="ID of the target table to place the cube on",
            ),
            place_action_client_node,
        ]
    )
