cmake_minimum_required(VERSION 3.0.2)
project(pallet_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pallet_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
