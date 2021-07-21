import ros_acoustics.robot as robot
robot = Robot

robot.init() # loads PRA service and rviz components

# sets the pose of the robot and updates rviz
robot.setPose(pos=[2, 1, 1], ori=[2, 1, 3, 2])

# OR
robot.getPoseFromGazebo()

# sends the current robot pose to PRA service and
# retrieves the simulated waveform
robot.getWaveform()

# interactive pyplot
robot.plotWaveform()

