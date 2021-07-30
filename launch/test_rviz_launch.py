from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameters_type import SomeParameters

path_to_room_mesh = '/home/tanmay/Projects/ros2_ws2/src/ros_acoustics/test/data/t_pipe.stl'

def generate_launch_description():
	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		# arguments='-d '
	)
	room_pub_node = Node(
		package='ros_acoustics',
		executable='room_publisher.py',
		output='screen',
		arguments=[path_to_room_mesh]
	)

	srv_node = Node(
		package='ros_acoustics',
		executable='srv_compute_waveforms.py',
		output='screen',
	)
	robot_node = Node(
		package='ros_acoustics',
		executable='test_robot.py',
		output='screen',
	) 

	return LaunchDescription([rviz_node, room_pub_node, srv_node, robot_node])

