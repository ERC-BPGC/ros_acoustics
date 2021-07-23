from visualization_msgs.msg import Marker
from rclpy.node import Node
import rclpy
import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
	"""
	Converts euler roll, pitch, yaw to quaternion (w in last place)
	quat = [x, y, z, w]
	Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
	"""
	cy = np.cos(yaw * 0.5)
	sy = np.sin(yaw * 0.5)
	cp = np.cos(pitch * 0.5)
	sp = np.sin(pitch * 0.5)
	cr = np.cos(roll * 0.5)
	sr = np.sin(roll * 0.5)

	q = [0] * 4
	q[0] = cy * cp * cr + sy * sp * sr
	q[1] = cy * cp * sr - sy * sp * cr
	q[2] = sy * cp * sr + cy * sp * cr
	q[3] = sy * cp * cr - cy * sp * sr

	return q

class Robot(Node):

	def __init__(self):
		super().__init__('robot')
		self.position = [0., 0., 0.]
		self.quaternion = [1., 0., 0., 0.]     # in radians

		self.pose_pub = self.create_publisher(Marker, 'robot_pose', 1)

	def publish_pose(self):
		m = Marker()
		m.pose.position.x = self.position[0]
		m.pose.position.y = self.position[1]
		m.pose.position.z = self.position[2]
		m.pose.orientation.x = self.quaternion[0]
		m.pose.orientation.y = self.quaternion[1]
		m.pose.orientation.z = self.quaternion[2]
		m.pose.orientation.w = self.quaternion[3]
		m.type = Marker.CUBE
		m.scale.x = m.scale.y = m.scale.z = 0.08
		m.action = Marker.ADD
		m.id = 69
		m.color.a = 1.0
		m.color.r = 0.7
		m.color.g = 0.1
		m.color.b = 0.15

		self.pose_pub.publish(m)
		

	def set_pose(self, position, rpy):
		self.position = list(position)
		q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
		self.quaternion = list(q)