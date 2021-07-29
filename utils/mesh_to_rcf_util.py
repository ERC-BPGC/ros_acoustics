#! /usr/bin/env python3

# hack to import ros_acoustics module
import pathlib
import sys
from matplotlib import interactive

from numpy.core.shape_base import block
parent_dir = pathlib.Path(sys.argv[0]).parent.absolute().parent.absolute().__str__()
sys.path.append(parent_dir)

from ros_acoustics.utils.pra_utils import ComplexRoom
import pyroomacoustics as pra
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d as a3
import os

def main():
	if len(sys.argv) != 3:
		# TODO: display help message 
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

	# TODO: obtain default wall material coefficients
	# print('Enter the default values for wall parameters:')
	# def_absorp = get_default_param('absorption')
	default_material = pra.Material(0.5, None)

	scale_factor = float(input('Input scale factor: '))

	try:
		room = ComplexRoom.from_stl(path_to_stl, default_material, scale_factor)
	except FileNotFoundError as f:
		print('Could not find mesh file: ' + path_to_stl + '. Exiting.')
		print(f)
		return
	except:
		print('Unknown error.')
		return

	# TODO: guide for ux
	# disp_guide_message()
	print(f'There are {len(room.walls)} walls. ')

	plt.ion() # TODO: close plot after for loop
	for wall in room.walls:
		widx = int(wall.name.split('_')[1])
		
		# TODO: don't keep changing camera angle of view
		room.plot_interactive(highlight_wall=widx, wireframe=False, interactive=True)

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
	room.save_rcf(path_to_rcf)

	test = input('Would you like to load the rcf for testing? (y/n)')

	if test == 'y':
		print('Loading rcf for testing...')
		room = ComplexRoom.from_rcf(path_to_rcf)
		plt.ioff()
		room.plot()
		plt.show()
	else:
		print('Test aborted')

	print('Done. Exiting.')

def get_wall_param(wall_name: str, param_name: str, default_value: float):
	"""Helper function to obtain a parameter from a user. Returns user entered value
	only if it is a valid float within (0,1), otherwise returns default_value

	Args:
		wall_name (str): Name of parameter's parent wall
		param_name (str): Name of parameter
		default_value (float): Value assigned if user gives invalid/skips

	Returns:
		[float]: Value of the parameter
	"""
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
