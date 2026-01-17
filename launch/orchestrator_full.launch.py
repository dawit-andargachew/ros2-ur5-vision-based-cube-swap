from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
	use_sim_time = LaunchConfiguration("use_sim_time")
	max_retries = LaunchConfiguration("max_retries")

	# MoveIt config for action servers
	moveit_config_pkg = "ir_movit_config"
	moveit_config = MoveItConfigsBuilder(
		"ir_gripper", package_name=moveit_config_pkg
	).to_moveit_configs()

	# 1. Launch AprilTag detection (silenced - runs in background)
	apriltag_launch = ExecuteProcess(
		cmd=['ros2', 'launch', 'group_6_assignment_2', 'apriltag_wrapper.launch.yml'],
		output='log',
		emulate_tty=False
	)

	# 2. Cube detector service (screen output)
	cube_detector_node = Node(
		package="group_6_assignment_2",
		executable="cube_detector",
		name="cube_detector",
		output="screen",
		parameters=[
			{"use_sim_time": use_sim_time},
		],
	)

	# 3. Pick action server (screen output for important logs)
	pick_action_server_node = Node(
		package="group_6_assignment_2",
		executable="pick_action_server",
		name="pick_action_server",
		output="screen",
		parameters=[
			{"use_sim_time": use_sim_time},
			PathJoinSubstitution(
				[FindPackageShare("group_6_assignment_2"), "config", "pick_params.yaml"]
			),
			moveit_config.to_dict(),
		],
	)

	# 4. Place action server (screen output for important logs)
	place_action_server_node = Node(
		package="group_6_assignment_2",
		executable="place_action_server",
		name="place_action_server",
		output="screen",
		parameters=[
			{"use_sim_time": use_sim_time},
			moveit_config.to_dict(),
		],
	)

	# 5. Orchestrator (delayed start to allow servers to initialize, screen output)
	orchestrator_node = TimerAction(
		period=5.0,  # Wait 5 seconds for all servers to start
		actions=[
			Node(
				package="group_6_assignment_2",
				executable="orchestrator",
				name="orchestrator",
				output="screen",
				parameters=[
					{"use_sim_time": use_sim_time},
					{"max_retries": max_retries},
				],
			)
		]
	)

	return LaunchDescription(
		[
			DeclareLaunchArgument(
				"use_sim_time",
				default_value="true",
				description="Use simulation clock if true",
			),
			DeclareLaunchArgument(
				"max_retries",
				default_value="5",
				description="Number of retry attempts per pick/place operation",
			),
			apriltag_launch,
			cube_detector_node,
			pick_action_server_node,
			place_action_server_node,
			orchestrator_node,
		]
	)
