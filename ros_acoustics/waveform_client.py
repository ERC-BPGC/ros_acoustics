#!/usr/bin/env python3
from ros_acoustics.srv import ComputeWaveforms

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

class WaveformClient(Node):
	"""TODO: description"""
	srv_name = 'compute_waveforms'
	
	def __init__(self):
		super().__init__('waveforms_client')
		self.cli = self.create_client(ComputeWaveforms, self.srv_name)
		self.get_logger().info('Initiated client.')
		
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service unavailable, trying again in 1s...')
		
		self.req = ComputeWaveforms.Request()

	def get_waveform(self, source_pos, mic_pos, source_wav):
		self.send_request(source_pos, mic_pos, source_wav)
		rclpy.spin_until_future_complete(self, self.future)

		try:
			response = self.future.result()
		except Exception as e:
			self.get_logger().info('Service call failed %r' % (e,))
			return False
		else:
			self.get_logger().info('Received response!')
			return response.waveform.array

	def send_request(self, source_pos, mic_pos, source_wav):
		self.req.source_pos.x = source_pos[0]
		self.req.source_pos.y = source_pos[1]
		self.req.source_pos.z = source_pos[2]
		self.req.mic_pos.x = mic_pos[0]
		self.req.mic_pos.y = mic_pos[1]
		self.req.mic_pos.z = mic_pos[2]
		self.req.source_wav.array = list(source_wav)

		self.future = self.cli.call_async(self.req)

