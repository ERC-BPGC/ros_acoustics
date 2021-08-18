#!/usr/bin/env python3
from ros_acoustics.srv import ComputeWaveforms

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

class AcousticsClient(Node):
	"""
	Class that handles sending and receiving requests from the 
	compute waveforms service
	"""
	srv_name = 'acoustics_service'
	
	def __init__(self):
		super().__init__('acoustics_client_node')
		self.cli = self.create_client(ComputeWaveforms, self.srv_name)
		self.get_logger().info('Initiated client.')
		
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Acoustics service unavailable, trying again in 1s...')
		
		self.req = ComputeWaveforms.Request()

	def get_waveform(self, source_pos, mic_pos, source_wav):
		"""(Warning: this function blocks i think!) Returns the simulated waveform
		obtained from compute waveforms service

		Args:
			source_pos (list of shape (3,)): position of sound source
			mic_pos (list of shape (3,)): position of microphone
			source_wav (1D list): waveform emitted by sound source

		Returns:
			list: simulated waveform
		"""
		self.send_request(source_pos, mic_pos, source_wav)
		rclpy.spin_until_future_complete(self, self.future)

		try:
			response = self.future.result()
		except Exception as e:
			self.get_logger().info('Service call failed %r' % (e,))
			return False
		else:
			self.get_logger().info('Received response!')
			return list(response.waveform.array)

	def send_request(self, source_pos, mic_pos, source_wav):
		self.req.source_pos.x = float(source_pos[0])
		self.req.source_pos.y = float(source_pos[1])
		self.req.source_pos.z = float(source_pos[2])
		self.req.mic_pos.x = float(mic_pos[0])
		self.req.mic_pos.y = float(mic_pos[1])
		self.req.mic_pos.z = float(mic_pos[2])
		self.req.source_wav.array = list(source_wav)

		self.future = self.cli.call_async(self.req)

