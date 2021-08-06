"""Tests ComplexRoom.from_bounding_box"""

# hack to import ros_acoustics module
import pathlib
import sys

from numpy.lib.utils import source
parent_dir = pathlib.Path(sys.argv[0]).\
				parent.absolute().\
				parent.absolute().\
				parent.absolute().\
				__str__()
sys.path.append(parent_dir)

from ros_acoustics.utils.pra_utils import BoundingBox, ComplexRoom, Limits
import pyroomacoustics as pra
import matplotlib.pyplot as plt
import numpy as np

reverse_normals = False

room_material = pra.Material(0.8, None)
room_bb = BoundingBox(
	x = Limits(0, 5),
	y = Limits(0, 4),
	z = Limits(0, 5),
)
room = ComplexRoom.from_bounding_box(
	room_bb,
	room_material
)

room.add_source((3, 3, 1))
room.add_microphone((3, 3, 2))

print('Volume: ', room.get_volume())
for w in room.walls:
	print(w.normal / np.linalg.norm(w.normal), w.corners[:,0], w.area())
	
# plot room
room.plot(img_order=1)
plt.show()

# for w in room.walls:
# 	print(w.corners)

print(room.get_volume())

# plot rir
room.compute_rir()
room.plot_rir()
plt.show()