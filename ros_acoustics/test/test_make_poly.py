# hack to import ros_acoustics module
import pathlib
import sys
parent_dir = pathlib.Path(sys.argv[0]).\
				parent.absolute().\
				parent.absolute().\
				parent.absolute().\
				__str__()
sys.path.append(parent_dir)

from ros_acoustics.utils.pra_utils import ComplexRoom
import pyroomacoustics as pra
import matplotlib.pyplot as plt

room_material = pra.Material(0.8, None)
room = ComplexRoom.make_polygon(
		material=room_material,
		centre=[0,0,0], 
		radius=5, 
		height=2, 
		N=3, 
		rpy=[0,0,0]
	)
room.plot()
plt.show()
