#!/usr/bin/env python3
"""
Simple test script that tests the functionality of the WaveformClient class
"""

from ros_acoustics.srv import ComputeWaveforms
from ros_acoustics.waveform_client import WaveformClient
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

def main(args=None):
	rclpy.init(args=None)

	waveforms_client = WaveformClient()

	source_pos = [3., .15, .15]
	mic_pos = [3.5, .15, .15]
	source_wav = [0.0]*500

	waveform = waveforms_client.get_waveform(source_pos, mic_pos, source_wav)

	plt.plot(waveform)
	plt.show()

	print('Exiting...')
	waveforms_client.destroy_node()

	rclpy.shutdown()

if __name__ == "__main__":
	main()
