cmake_minimum_required(VERSION 3.5)
project(ros_acoustics)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sprintbot-model REQUIRED)

# no build testing yet

# required for python nodes
ament_python_install_package(${PROJECT_NAME})

# install launch file
install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# install python scripts
set(python_scripts
  ros_acoustics/acoustics_service.py
  ros_acoustics/odom_subscriber.py
  ros_acoustics/room_publisher.py
  test/test_basic.py
)
install(PROGRAMS
  ${python_scripts}
  DESTINATION lib/${PROJECT_NAME}
)

# install our custom interfaces
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/Point3D.msg
  msg/Point3DArray.msg
  msg/FloatArray.msg
)
set(srv_files
  srv/ComputeWaveforms.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

# install data files (STLs and RCFs)
install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
