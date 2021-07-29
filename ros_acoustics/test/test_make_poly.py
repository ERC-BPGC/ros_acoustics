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

from ros_acoustics.utils.pra_utils import ComplexRoom
import pyroomacoustics as pra
import matplotlib.pyplot as plt

reverse_normals = False

room_material = pra.Material(0.8, None)
room = ComplexRoom.make_polygon(
		material=room_material,
		centre=[0,0,0], 
		radius=5, 
		height=2, 
		N=3, 
		rpy=[0,0,0],
		reverse_normals=reverse_normals
	)

source_pos = [1,0,.3]
room.add_source(source_pos)
room.add_microphone([0,0,0.5])
room.compute_rir()

# plot room
room.plot(img_order=1)
plt.show()

# plot rir
room.plot_rir()
plt.show()
