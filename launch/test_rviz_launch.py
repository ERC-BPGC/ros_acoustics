from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameters_type import SomeParameters

def generate_launch_description():
	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		# arguments='-d '
	)
	robot_node = Node(
		package='ros_acoustics',
		executable='test_robot.py',
	)

	return LaunchDescription([rviz_node, some_node])

