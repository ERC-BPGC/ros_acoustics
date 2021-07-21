#! /usr/bin/env python3
from numpy.core.numeric import load
from ros_acoustics.srv import ComputeWaveforms
import pyroomacoustics as pra
import numpy as np
import rclpy
from rclpy.node import Node

from ros_acoustics.file_utils import *
from ros_acoustics.room_utils import *

class ComputeWaveformsService(Node):
	# TODO: Docstring
	def __init__(self, room_path):
		super().__init__('compute_waveforms_service')
		self.srv = self.create_service(ComputeWaveforms, 'compute_waveforms', self.compute_waveforms_callback)
		
		# TODO: Set these variables using parameter server
		self._room_path = room_path
		self._room_fs = 8000
		self._room_air_absorption = True
		self._room_ray_tracing = False
		
		self._info_log('Initiated compute waveforms service.')

	def compute_waveforms_callback(self, request, response):
		self._info_log('Received request to compute waveforms.')

		room = load_room(self._room_path)
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
	
def main(args=None):
	rclpy.init(args=None)

	# TODO: room path from command line arg
	room_path = "/home/tanmay/Projects/echoslam_acoustics/ros_acoustics/data/room/t_pipe.yaml"
	compute_waveforms_service = ComputeWaveformsService(room_path)

	rclpy.spin(compute_waveforms_service)

if __name__ == '__main__':
	main()