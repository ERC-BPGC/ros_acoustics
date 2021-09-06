#!/usr/bin/env python3
from ros_acoustics.acoustics_client import AcousticsClient
import rclpy
from visualization_msgs.msg import Marker
import time
import numpy as np
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker

def main(args=None):
	rclpy.init(args=None)
	time.sleep(1.0)
	print('Initiated robot test script.')

	robot_pos = np.array([3, 1, 1.])
	source_rel_pos = np.array([0,0,0.])
	mic_rel_pos = np.array([0,0,0.1])


	rm_node = rclpy.create_node('robot_marker')
	rm_pub = rm_node.create_publisher(Marker, 'robot_marker', 1)
	rm_pub.publish(gen_robot_marker(robot_pos, rm_node.get_clock().now().to_msg()))
	source_wav = gen_source_wav_demo()

	waveform_client = AcousticsClient()
	computed_waveform = waveform_client.get_waveform(
		source_pos=robot_pos+source_rel_pos,
		mic_pos=robot_pos+mic_rel_pos,
		source_wav=source_wav,
	)
	fig, axs = plt.subplots(2)
	axs[0].plot(source_wav)
	axs[1].plot(computed_waveform)
	plt.show()

	rclpy.shutdown()

def gen_source_wav_demo():
	x = np.zeros(8000) # one second (fs=8000hz) long delta impulse
	x[1:3] += 1.
	return x

def gen_robot_marker(robot_pos, timestamp):
	m = Marker()
	m.header.frame_id = "map"
	m.header.stamp = timestamp
	m.pose.position.x = float(robot_pos[0])
	m.pose.position.y = float(robot_pos[1])
	m.pose.position.z = float(robot_pos[2])
	m.pose.orientation.x = 1.
	m.pose.orientation.y = 0.
	m.pose.orientation.z = 0.
	m.pose.orientation.w = 0.
	m.type = Marker.CUBE
	m.scale.x = .2
	m.scale.y = .2
	m.scale.z = .2
	m.action = Marker.ADD
	m.id = 69
	m.color.a = 1.
	m.color.r = 0.1
	m.color.g = 0.9
	m.color.b = 0.2

	return m 

if __name__ == '__main__':
	main()