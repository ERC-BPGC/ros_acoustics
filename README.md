# The ROS Acoustics Package

## Instructions for use

1. Create a valid ROS2 workspace using colcon.
2. Clone this repo inside your `src` folder.
3. Go to the top workspace directory and build using `colcon build --packages-select ros_acoustics`
4. Source the workspace: `source install/local_setup.bash`
5. To run the basic test: `ros2 launch ros_acoustics basic_test_launch.py`  