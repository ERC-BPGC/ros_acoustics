# hack to import ros_acoustics module
import pathlib
import sys
parent_dir = pathlib.Path(sys.argv[0]).\
				parent.absolute().\
				parent.absolute().\
				__str__()
sys.path.append(parent_dir)

from ros_acoustics.pra_utils.complex_room import ComplexRoom
import matplotlib.pyplot as plt
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('rcf', help='File path to room config file')
args = parser.parse_args()

room = ComplexRoom.from_rcf(args.rcf)
room.plot()
plt.show()