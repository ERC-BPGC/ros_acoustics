#!/usr/bin/env python3
from ros_acoustics.acoustics_client import AcousticsClient
import rclpy
import numpy as np
import matplotlib.pyplot as plt

def main():
	rclpy.init()

	robot_pos = np.array([5.,3.,.11])
	mic_pos = robot_pos + np.array([0,0,0.1])
	source_pos = robot_pos + np.array([0,0,0])
	fs = 16000
	# wav = gen_waveform(fs, 100, 1)
	wav = np.array((0., 1., 0.))
	acoustics_client = AcousticsClient()

	simwav = acoustics_client.get_waveform(source_pos, mic_pos, wav)
	dists = np.linspace(0, len(simwav)*343/fs, len(simwav))

	plt.plot(dists, simwav)
	plt.show()
	

def gen_waveform(fs, freq, length):
	x = np.linspace(0, length, fs*length)
	wav = np.sin(2*np.pi*freq*x)
	return wav

# def gen_impulse(fs)

if __name__ == "__main__":
	main()