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
	if len(sys.argv) != 3:
		print('Need 2 args - path to mesh existing mesh file, and path of outputted rcf. Exiting.')
		return

	# clarify the user with the cwd
	os.chdir(parent_dir + '/test/data')
	print('Current working directory: ' + os.getcwd())

	path_to_stl = sys.argv[1]
	path_to_rcf = sys.argv[2]

	# check extension of input mesh file
	if path_to_stl.split('.')[1].lower() != 'stl':
		print('Currently only STL files supported. Exiting. ')
		return 

	# obtain default wall material coefficients
	print('Enter the default values for wall parameters:')
	# def_absorp = get_default_param('absorption')
	default_material = pra.Material(0.5, None)

	try:
		room = pra_utils.stl_to_room(path_to_stl, default_material)
	except FileNotFoundError:
		print('Could not find mesh file: ' + path_to_stl + '. Exiting.')
		return
	except:
		print('Unknown error.')
		return

	# disp_help_message()

	plt.ion()
	for wall in room.walls:
		widx = int(wall.name.split('_')[1])
		
		pra_utils.plot_room(room, highlight_wall=widx, wireframe=False, interactive=True)

		# ask user for values for wall
		absorption = get_wall_param(wall.name, 'energy_absorption', 0.5)
		scattering = get_wall_param(wall.name, 'scattering', None)
		wall = pra.wall_factory(
			wall.corners,
			[absorption],
			[scattering],
			name=wall.name
		)
	
	print(f'Saving file to {path_to_rcf}')
	pra_utils.dump_room(room, path_to_rcf)

	test = input('Would you like to load the rcf for testing? (y/n)')
	test = True if test == 'y' else False

	if test:
		print('Loading rcf for testing...')
		room = pra_utils.load_room(path_to_rcf)
		plt.ioff()
		room.plot()
		plt.show()
	else:
		print('Test aborted')

	print('Done. Exiting.')

def get_wall_param(wall_name: str, param_name: str, default_value: float):
	p = input(f'Enter {param_name} for {wall_name}: ')

	# if user presses enter, use default value
	if p.strip() == "":
		print(
			f'Assigning default {param_name} '
			f'({default_value})'
			f' for {wall_name}'
		)
		p = default_value
	else:
		try:
			p = float(p)
			if not(0.0 < p < 1.0):
				raise ValueError
		except:
			print(
			f'Invalid value! Assigning default {param_name} '
			f'({default_value})'
			f' for {wall_name}'
			)
			p = default_value

	return p
	

if __name__ == "__main__":
	main()
