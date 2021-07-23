#!/usr/bin/env python3
import sys
print(sys.path)
from ros_acoustics.srv import ComputeWaveforms

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker

# class MarkerPublisher(Node):
# 	def __init__(self, topic):
# 		super().__init__('minimal_publisher')
# 		self.publisher_ = self.create_publisher(Marker, topic, 10)

# 	def publish(self, pos):
# 		marker_msg = Marker
# 		marker_msg.pose.position.x = pos[0]
# 		marker_msg.pose.position.x = pos[0]
# 		marker_msg.pose.position.x = pos[0]
# 		marker_msg.type = Marker.CUBE
# 		self.publisher_.publish(marker_msg)

class BasicComputeWaveformsClient(Node):
	
	def __init__(self):
		super().__init__('basic_waveforms_client')
		self.cli = self.create_client(ComputeWaveforms, 'compute_waveforms')
		self.get_logger().info('Initiated client.')
		
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service unavailable, trying again in 1s...')
		
		self.req = ComputeWaveforms.Request()
		self.waveform = np.sin(np.linspace(0, 0.5, 8000))

	def send_request(self, source_pos, mic_pos):
		self.req.source_pos.x = source_pos[0]
		self.req.source_pos.y = source_pos[1]
		self.req.source_pos.z = source_pos[2]
		self.req.mic_pos.x = mic_pos[0]
		self.req.mic_pos.y = mic_pos[1]
		self.req.mic_pos.z = mic_pos[2]
		self.req.source_wav.array = list(self.waveform)

		self.future = self.cli.call_async(self.req)

def main(args=None):
	rclpy.init(args=None)

	waveforms_client = BasicComputeWaveformsClient()

	source_pos = [3., .15, .15]
	mic_pos = [3.5, .15, .15]
	waveforms_client.send_request(source_pos, mic_pos)

	while rclpy.ok():
		rclpy.spin_once(waveforms_client)

		if waveforms_client.future.done():
			try:
				response = waveforms_client.future.result()
			except Exception as e:
				waveforms_client.get_logger().info(
					'Service call failed %r' % (e,))
			else:
				waveforms_client.get_logger().info('Received response! Plotting...')

				plt.plot(response.waveform.array)
				plt.show()

			break

	print('Exiting...')
	waveforms_client.destroy_node()

	rclpy.shutdown()

if __name__ == "__main__":
	main()
