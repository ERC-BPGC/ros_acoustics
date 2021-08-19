# The ROS Acoustics Package

## Running a basic acoustics demo with Gazebo

1. Create a valid ROS2 workspace using colcon.
2. Clone this repo inside your `src` folder.
3. Add sprintbot-model and optionally key_teleop packages in your `src` folder from [here](https://github.com/SheldonLee123/PipeWorld/tree/main/src)
4. Build these 3 packages using `colcon build`.
5. Source the workspace: `source install/setup.bash`
6. Launch: `ros2 launch ros_acoustics gazebo_test_launch.py`. You should be able to see gazebo with a sprintbot and some objects.
7. Optionally, you can teleop sprintbot by running this in another terminal: `ros2 run key_teleop key_teleop`

## File Description

