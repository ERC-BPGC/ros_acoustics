#!/usr/bin/env python3
from ros_acoustics.robot import Robot
import rclpy

def main(args=None):
	rclpy.init(args=None)

	robot = Robot()
	robot.set_pose([1., 0.075, 0.075], [0,0,0])
	robot.publish_pose()

	rclpy.spin(robot)

	robot.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()