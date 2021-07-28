#!/usr/bin/env python3
import sys

from numpy.core.numeric import load
print(sys.path)
sys.path.append('/home/tanmay/Projects/ros2_ws2/src/ros_acoustics')
import os
print(os.getcwd())
import rclpy

from utils.pra_utils import *
import matplotlib.pyplot as plt

rcf_path = 'test/data/simple_pipe_from_stl.yaml'
stl_path = 'test/data/simple_pipe.stl'

room = stl_to_room(stl_path)
room.plot()
fix_plt_axs(plt, [-2, 10], [-2, 10])
plt.show()

dump_room(room, rcf_path)

room = load_room(rcf_path)
room.plot()
fix_plt_axs(plt, [-2, 10], [-2, 10])
plt.show()
