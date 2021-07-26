#!/usr/bin/env python3
from ros_acoustics.robot import Robot
import rclpy
from visualization_msgs.msg import Marker

def main(args=None):
	rclpy.init(args=None)

	# publish_stl('test/data/simple_pipe.stl')

	robot = Robot()
	robot.set_pose([1., 0.075, 0.075], [0,0,0])
	robot.publish_pose()

	rclpy.spin(robot)

	robot.destroy_node()
	rclpy.shutdown()

def publish_stl(path_to_stl):
	m = Marker()
	m.pose.position.x = 0
	m.pose.position.y = 0
	m.pose.position.z = 0
	m.pose.orientation.x = 0
	m.pose.orientation.y = 0
	m.pose.orientation.z = 0
	m.pose.orientation.w = 1.0
	m.type = Marker.MESH_RESOURCE
	m.scale.x = m.scale.y = m.scale.z = 1.
	m.action = Marker.ADD
	m.id = 42
	m.color.a = 0.6
	m.color.r = 0.1
	m.color.g = 0.2
	m.color.b = 0.75

if __name__ == '__main__':
	main()