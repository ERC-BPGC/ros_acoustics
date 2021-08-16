#! /usr/bin/env python3
from ros_acoustics.srv import ComputeWaveforms
from ros_acoustics.utils.pra_utils import *
import rclpy
from rclpy.node import Node

import pyroomacoustics as pra
import numpy as np
import os, argparse

class AcousticsService(Node):
	"""Class that runs the acoustics simulations as a ros2 service"""

	def __init__(self, path_to_rcf):
		"""Constructor

		Args:
			path_to_rcf (str): File path to the room config file
		"""
		super().__init__('acoustics_service_node')
		self.srv = self.create_service(ComputeWaveforms, 'acoustics_service', self.compute_waveforms_callback)

		# retrieve room parameters
		# self._room_mesh_path = path_to_mesh
		self._room_rcf_path = path_to_rcf
		self._room_fs = 8000
		self._room_air_absorption = True
		self._room_ray_tracing = False

		# TODO: Set these variables using parameter server
		
		self._info_log(f'Initiated compute waveforms service.{os.getcwd()}')

	def compute_waveforms_callback(self, request, response):
		"""Runs an acoustic room simulation upon receiving a request.

		Args:
			request (ComputeWaveforms.request): 
			response (ComputeWaveforms.response): 

		Returns:
			[List[Float]]: The simulated mic waveform
		"""
		self._info_log('Received request to compute waveforms.')

		room = ComplexRoom.from_rcf(self._room_rcf_path)
		room.add_source(
			position=[request.source_pos.x, request.source_pos.y, request.source_pos.z],
			signal=np.array(request.source_wav.array),
		)
		room.add_microphone(
			loc=[request.mic_pos.x, request.mic_pos.y, request.mic_pos.z]
		)
		room.simulate()

		response.waveform.array = list(room.mic_array.signals[0])
		return response

	def _info_log(self, info_msg):
		self.get_logger().info(info_msg)

def main():
	rclpy.init(args=None)

    # manage cli args
	parser = argparse.ArgumentParser()
	parser.add_argument('--rcf', required=True,
						help='Path to the room rcf file')
	parser.add_argument('--mesh',
						help='Path to the room mesh file, to display in rviz')
	args = parser.parse_args()

	compute_waveforms_service = AcousticsService(args.rcf)

	rclpy.spin(compute_waveforms_service)

if __name__ == '__main__':
	main()