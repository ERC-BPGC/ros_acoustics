from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
	# Getting paths
	gazebo_path = get_package_share_directory('gazebo_ros')
	acoustic_path = get_package_share_directory('ros_acoustics')
	world_path = acoustic_path + '/worlds/simple_pipe.world'
	urdf_path = get_package_share_directory('sprintbot-model') + '/urdf/sprintbot.urdf'
	rcf_path = acoustic_path + '/data/simple_pipe_bounded.rcf'
	# initial position of sprintbot
	bot_initpos = [str(c) for c in [5., -1, 0.11]]

	params = {'robot_description' : open(urdf_path).read()}
	robot_spawner = Node(package='gazebo_ros',
						executable='spawn_entity.py',
						arguments=[
							'-topic', 'robot_description', 
							'-entity', LaunchConfiguration('rb_id'), 
							'-robot_namespace', LaunchConfiguration('rb_id'), 
							'-z', bot_initpos[2],
							'-y', bot_initpos[1],
							'-x', bot_initpos[0],
						],
						output='screen')
	robot_state_publisher = Node(package='robot_state_publisher',
								 executable='robot_state_publisher',
								 output='screen',
								 parameters=[params])

	# acoustics service node
	acoustic_srv_node = Node(
		package='ros_acoustics',
		executable='acoustics_service.py',
		arguments=['--rcf', rcf_path],
		output='screen',
	)
	# basic client
	acoustic_cli_node = Node(
		package='ros_acoustics',
		executable='test_basic.py',
	)

	return LaunchDescription([
		DeclareLaunchArgument(
			'world', default_value=world_path,
			description='The path to the world file to load.'
		),
		DeclareLaunchArgument(
			'rb_id', default_value='sprintbot',
			description='The name of the robot to spawn.'
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([gazebo_path, '/launch/gazebo.launch.py'])
		),
		robot_state_publisher,
		robot_spawner,
		acoustic_srv_node,
		# acoustic_cli_node,
	])