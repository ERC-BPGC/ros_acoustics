#! /usr/bin/env python3

# hack to import ros_acoustics module
import pathlib
import sys
from matplotlib import interactive

from numpy.core.shape_base import block
parent_dir = pathlib.Path(sys.argv[0]).parent.absolute().parent.absolute().__str__()
sys.path.append(parent_dir)

from ros_acoustics.utils import pra_utils
import pyroomacoustics as pra
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d as a3
import os

def main():
	if len(sys.argv) != 2:
		print('Path to mesh file required as argument. Exiting.')
		return

	# clarify the user with the cwd
	os.chdir(parent_dir + '/test/data')
	print('Current working directory: ' + os.getcwd())

	path_to_stl = sys.argv[1]

	# check extension of input mesh file
	if path_to_stl.split('.')[1].lower() != 'stl':
		print('Currently only STL files supported. Exiting. ')
		return 

	# obtain default wall material coefficients
	default_material = pra.Material(0.5, None)

	try:
		room = pra_utils.stl_to_room(path_to_stl, default_material)
	except FileNotFoundError:
		print('Could not find mesh file: ' + path_to_stl + '. Exiting.')
		return
	except:
		print('Unknown error.')
		return
	

	plt.ion()
	for wall in room.walls:
		widx = int(wall.name.split('_')[1])
		
		pra_utils.plot_room(room, highlight_wall=widx, wireframe=False, interactive=True)
		
		absorption = input(f'Enter energy absorption for {wall.name}: ')
		if absorption.strip() == "":
			print(
				f'Assigning default energy absorption '
				f'({default_material.absorption_coeffs})'
				f' for wall {wall.name}'
			)
			absorption = default_material.absorption_coeffs[0]
		else:
			try:
				absorption = float(absorption)
				if not(0.0 < absorption < 1.0):
					raise ValueError
			except:
				print(
				f'Invalid value! Assigning default energy absorption '
				f'({default_material.absorption_coeffs})'
				f' for wall {wall.name}'
				)
		wall = pra.wall_factory(
			wall.corners,
			[absorption],
			default_material.scattering_coeffs,
			name=wall.name
		)
	
	pra_utils.dump_room(room, 'testconvert.yaml')

	room = pra_utils.load_room('testconvert.yaml')
	plt.ioff()
	room.plot()
	plt.show()

if __name__ == "__main__":
	main()
