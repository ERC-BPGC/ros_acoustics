from ros_acoustics.acoustics_client import AcousticsClient
import rclpy

def main():
	robot_pos = np.array([1,1,1.])
	mic_pos = robot_pos + np.array([0,0,0.1])
	source_pos = robot_pos

if __name__ == "__main__":
	main()