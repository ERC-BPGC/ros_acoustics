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
    world_path = acoustic_path + '/worlds/t_pipe.world'
    urdf = get_package_share_directory('sprintbot-model') + '/urdf/sprintbot.urdf'
    y = -5. # TODO: initial coordinates in vars
    
    params = {'robot_description' : open(urdf).read()}
    robot_spawner = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=[
                            '-topic', 'robot_description', 
                            '-entity', LaunchConfiguration('rb_id'), 
                            '-robot_namespace', LaunchConfiguration('rb_id'), 
                            '-z','0.11',
                            '-y', str(y),
                            '-x', '2.3'
                        ],
                        output='screen')
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='screen',
                                 parameters=[params])

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
        robot_spawner
    ])