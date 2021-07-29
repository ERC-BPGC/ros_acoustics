#!/usr/bin/env python3
from ros_acoustics.srv import ComputeWaveforms

import rclpy
from rclpy.node import Node
import numpy as np
import time
from visualization_msgs.msg import Marker
import sys, os

class RoomPublisher(Node):
	"""Class that publishes the marker for a room"""
	topic = 'room_marker'
	# path_to_mesh = 'file:///home/tanmay/Projects/ros2_ws2/src/ros_acoustics/test/data/t_pipe.stl'

	def __init__(self, path_to_mesh):
		super().__init__('room_publisher')

		self.path_to_mesh = path_to_mesh
		self.pub = self.create_publisher(Marker, self.topic, 2)
		self.marker = self._init_basic_marker()
		self.marker.mesh_resource = 'file://' + self.path_to_mesh

		self.get_logger().info('Initiated room publisher')

	def publish_room(self):
		self.get_logger().info('Publishing room marker.')
		self.pub.publish(self.marker)

	def _init_basic_marker(self):
		"""Returns basic marker"""
		m = Marker()
		m.header.frame_id = "map"
		m.header.stamp = self.get_clock().now().to_msg()
		m.pose.position.x = 0.
		m.pose.position.y = 0.
		m.pose.position.z = 0.
		m.pose.orientation.x = 1.
		m.pose.orientation.y = 0.
		m.pose.orientation.z = 0.
		m.pose.orientation.w = 0.
		m.type = Marker.MESH_RESOURCE
		m.scale.x = m.scale.y = m.scale.z = 1.
		m.action = Marker.ADD
		m.id = 42
		m.color.a = 0.6
		m.color.r = 0.1
		m.color.g = 0.2
		m.color.b = 0.75

		return m 

def main(args=None):
	if sys.argv[1].split('.')[1] not in ('stl'):
		print('Needs 1 positional arg for path to room mesh. Aborting.')
		return
	
	path_to_mesh = sys.argv[1]
	if not os.path.isfile(path_to_mesh):
		print('Mesh file cannot be found. Aborting.')
		return

	rclpy.init(args=None)
	time.sleep(3.0)	# TODO: this is a hack, fix lathcing problem

	room_publisher = RoomPublisher(path_to_mesh)
	room_publisher.publish_room()

	rclpy.spin(room_publisher)

	rclpy.shutdown()

if __name__ == '__main__':
	main()