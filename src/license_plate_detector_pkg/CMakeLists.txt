cmake_minimum_required(VERSION 3.0.2)
project(license_plate_detector_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  src/image_publisher.py
  src/license_plate_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
