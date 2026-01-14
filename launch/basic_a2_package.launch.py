from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
	use_sim_time = LaunchConfiguration("use_sim_time")

	# Do exactly the MoveIt config style from the example scaffold.
	moveit_config_pkg = "ir_movit_config"
	moveit_config = MoveItConfigsBuilder(
		"ir_gripper", package_name=moveit_config_pkg
	).to_moveit_configs()

	# apriltag_wrapper = IncludeLaunchDescription(
	# 	AnyLaunchDescriptionSource(
	# 		PathJoinSubstitution(
	# 			[FindPackageShare("group_6_assignment_2"), "launch", "apriltag_wrapper.launch.yml"]
	# 		)
	# 		output='log',
	# 		emulate_tty=False
	# 	)
	# )

	# 2. Launch apriltag detection node (silenced in terminal)
	apriltag_launch = ExecuteProcess(
		cmd=['ros2', 'launch','group_6_assignment_2', 'apriltag_wrapper.launch.yml'],
		output='log',
		emulate_tty=False
	)

	cube_detector_node = Node(
		package="group_6_assignment_2",
		executable="cube_detector",
		name="cube_detector",
		output="screen",
		parameters=[
			{"use_sim_time": use_sim_time},
		],
	)

	pick_action_server_node = Node(
		package="group_6_assignment_2",
		executable="pick_action_server",
		name="pick_action_server",
		output="screen",
		parameters=[
			{"use_sim_time": use_sim_time},
			PathJoinSubstitution(
				[FindPackageShare("group_6_assignment_2"), "config", "pick_action_server.yaml"]
			),
			PathJoinSubstitution(
				[FindPackageShare("group_6_assignment_2"), "config", "robot_target_positions.yaml"]
			),
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
			apriltag_launch,
			cube_detector_node,
			pick_action_server_node,
		]
	)
