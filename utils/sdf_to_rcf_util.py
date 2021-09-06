# hack to import ros_acoustics module
import pathlib
import sys
parent_dir = pathlib.Path(sys.argv[0]).parent.absolute().parent.absolute().__str__()
sys.path.append(parent_dir)

from ros_acoustics.pra_utils.complex_room import ComplexRoom
import numpy as np
import argparse
import xml.etree.ElementTree as et

def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('path_to_sdf',
		help='The path to an existing .sdf file')
	parser.add_argument('path_to_rcf',
		nargs='?',
		default=None,
		help='The path to save the generated room config file.')
	args = parser.parse_args()

	print('Reading SDF file: ', args.path_to_sdf)
	sdftree = et.parse(args.path_to_sdf)
	root = sdftree.getroot()
	print(sdftree)

if __name__ == "__main__":
	main()