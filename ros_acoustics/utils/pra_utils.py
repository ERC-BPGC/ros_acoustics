import numpy as np
import pyroomacoustics as pra
from pyroomacoustics import room
import yaml
import pprint as pp
from stl import mesh

## file utils

def room_to_stl(room: pra.Room, outpath: str) -> None:
	"""Converts a pyroomacoustics room object into an stl file

	Args:
		room (pyroomacoustics.Room): [description]
		outpath (string): Path of the outputted stl file
	"""
	num_walls = len(room.walls)
	room_mesh = mesh.Mesh(np.zeros(num_walls*2, dtype=mesh.Mesh.dtype))
	print(num_walls)
	for widx, wall in enumerate(room.walls):
		corners = wall.corners.T
		room_mesh.vectors[2*widx] = np.array([
			corners[0], corners[1], corners[2]
		])
		room_mesh.vectors[2*widx+1] = np.array([
			corners[0], corners[2], corners[3]
		])
		if widx >= 8:
			break
	# print(room_mesh.is_closed())
	room_mesh.save(outpath)

def stl_to_room(path_to_stl: str, material: pra.Material = None) -> pra.Room:
	# TODO: Other room params like fs in args/kwargs?
	material = pra.Material(0.5, None) if material is None else material

	room_mesh = mesh.Mesh.from_file(path_to_stl)
	ntriang = room_mesh.vectors.shape[0]
	scale_factor = 1.0	# TODO: as arg/kwarg?

	walls = []
	for i in range(ntriang):
		walls.append(
			pra.wall_factory(
				room_mesh.vectors[i].T * scale_factor,
				material.energy_absorption['coeffs'],
				material.scattering['coeffs'],
			)
		)

	room = pra.Room(walls, fs=16000, max_order=4, ray_tracing=False)

	return room


def load_room(inpath):
	with open(inpath, 'r') as file:
		# get room dict
		rdin = yaml.load(file, Loader=yaml.FullLoader)
	
	walls = []
	for w in rdin['walls']:
		# TODO: checks for value and type of attributes
		wcorners = np.array(w['corners']).T
		mat = pra.Material(w['material']['absorption'], w['material']['scattering'])

		walls.append(
			pra.wall_factory(
				wcorners,
				mat.energy_absorption['coeffs'],
				mat.scattering['coeffs']
			)
		)

	room = pra.Room(walls,
					fs=rdin['fs'],
					max_order=rdin['max_order'],
					air_absorption=rdin['air_absorption'],
					ray_tracing=rdin['ray_tracing']	
				)
	
	return room

def dump_room(room, outpath):
	"""TODO: Docstring"""
	rd = create_room_dict(room)
	with open(outpath, 'w') as file:
		yaml.dump(rd, file, default_flow_style=False, sort_keys=False)
	
	print('Done creating YAML dump.')

def print_room(room):
	"""Nicely prints details ab pra.Room object"""
	pp.pprint(create_room_dict(room), sort_dicts=False)

def create_room_dict(room: pra.Room): 
	rd = dict()     # room_dict
	rd['ray_tracing'] = room.simulator_state['rt_needed']
	rd['air_absorption'] = room.simulator_state['air_abs_needed']
	rd['max_order'] = room.max_order
	rd['fs'] = room.fs

	rd['walls'] = []
	for widx, wall in enumerate(room.walls):
		wd = dict() # dict for a single wall
		wd['id'] = widx
		wd['material'] = {
			'absorption': float(wall.absorption[0]),
			'scattering': float(wall.scatter[0]) if wall.scatter > 0 else None
		}
		
		# TODO: How to determine normal is reversed
		# wd['reverse_normal'] = False

		corners = wall.corners.T.tolist()
		wd['corners'] = []
		for corner in corners:
			wd['corners'].append(corner)

		rd['walls'].append(wd)

	return rd

## room utils

def create_walls(obstacle_faces, materials):
	"""Returns a list of Wall objects that can be used in the Room constructor

	Args:
		obstacle_faces (array/list of Nx3 numpy 2D matrices): Same as the output of make_polygon. 
			Each element of this is a 2D matrix representing a wall, each row is a coordinate
			of a vertex.
		material (pra.Material): Material of the wall

	Returns:
		list of Wall objects: Can be directly use in Room constructor
	"""
	material_list = False 
	if isinstance(materials, list):
		if len(materials) != len(obstacle_faces):
			raise TypeError("list of materials should be same length as obstacle_faces")
		else:
			material_list = True 
	elif not isinstance(materials, pra.Material):
		raise TypeError("materials should be pra.Material or list of pra.Material")

	walls = []
	for i in range(len(obstacle_faces)):
		m = materials[i] if material_list else materials
		walls.append(
			pra.wall_factory(
				obstacle_faces[i].T, 
				m.energy_absorption["coeffs"], 
				m.scattering["coeffs"]
			)
		)

	return walls 

def make_polygon(centre, radius, height, N=3, rpy=[0,0,0], reverse_normals=False):
	"""Create an extruded polygon

	Args:
		centre (array-like): centre of mass of polygon
		radius (float): distance from centre to side edge
		height (float): height of extrusion
		N (int, optional): Number of sides. Defaults to 3.
		rpy (array-like, optional): Roll, pitch and yaw (in that order). Defaults to [0,0,0].
		reverse_normals (bool, optional): If true, normals point inward. Keep true for obstacles, false for rooms.
			Defaults to False.

	Returns:
		list of 2D Nx3 numpy matrices (N>2): Each 2D numpy matrix is a row-wise list array of 
			coordinates of each vertex of a wall.
	"""
	lower_points = []
	upper_points = []
	
	for n in range(N):
		x = radius * np.cos(2*np.pi*n/N)
		y = radius * np.sin(2*np.pi*n/N)

		lower_points.append(np.array([x, y, height/2]))
		upper_points.append(np.array([x, y, -height/2]))

	# do rotation and translation
	cr, cp, cy = np.cos(rpy)
	sr, sp, sy = np.sin(rpy)
	Rx = np.array([
		[1, 0, 0],
		[0, cr, -sr],
		[0, sr, cr]
	]).T
	Ry = np.array([
		[cp, 0, sp],
		[0, 1, 0],
		[-sp, 0, cp]
	]).T
	Rz = np.array([
		[cy, -sy, 0],
		[sy, cy, 0],
		[0, 0, 1]
	]).T
	lower_points = np.array(lower_points) @ Rx @ Ry @ Rz + np.array(centre)
	upper_points = np.array(upper_points) @ Rx @ Ry @ Rz + np.array(centre)

	walls = []
	# add side walls
	for i in range(N-1):
		wall = np.array([
				lower_points[i], upper_points[i],
				upper_points[i+1], lower_points[i+1]
			])
		wall = wall[::-1] if reverse_normals else wall
		walls.append(wall)

	# last side wall
	wall = np.array([
				lower_points[N-1], upper_points[N-1],
				upper_points[0],lower_points[0] 
			])
	wall = wall[::-1] if reverse_normals else wall
	walls.append(wall)

	if reverse_normals:
		lower_points = lower_points[::-1]
		upper_points = upper_points[::-1]

	# lower and upper walls
	walls.append(np.array(lower_points))
	walls.append(np.array(upper_points[::-1]))

	return walls

def make_cylinder(centre, radius, height, rpy=[0,0,0], N=100):
	return make_polygon(centre, radius, height, N=N, rpy=rpy) 

def fix_plt_axs(plt, xylims, zlims):
	plt.gca().set_xlim3d(left=xylims[0], right=xylims[1])
	plt.gca().set_ylim3d(bottom=xylims[0], top=xylims[1])
	plt.gca().set_zlim3d(bottom=zlims[0], top=zlims[1])