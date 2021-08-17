from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	ra_path = get_package_share_directory('ros_acoustics')
	path_to_rcf = ra_path + '/data/simple_pipe.rcf'

	srv_parameters = [
		{'fs': 16000},
		{'ray_tracing': 'False'},
	]
	srv_node = Node(
		package='ros_acoustics',
		executable='acoustics_service.py',
		arguments=['--rcf', path_to_rcf],
		output='screen',
		# parameters=srv_parameters,
	)
	cli_node = Node(
		package='ros_acoustics',
		executable='test_basic.py',
	)

	return LaunchDescription([
		srv_node, 
		cli_node
	])

