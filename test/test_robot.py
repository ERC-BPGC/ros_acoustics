#!/usr/bin/env python3
from ros_acoustics.room_publisher import RoomPublisher
import rclpy
from visualization_msgs.msg import Marker
import time
def main(args=None):
	rclpy.init(args=None)
	time.sleep(3.0)	# TODO: this is a hack, fix lathcing problem

	room_publisher = RoomPublisher()
	room_publisher.publish_room()

	rclpy.spin(room_publisher)

	rclpy.shutdown()

if __name__ == '__main__':
	main()