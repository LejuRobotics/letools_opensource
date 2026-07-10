cmake_minimum_required(VERSION 3.0.2)
project(dynamic_biped)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  apriltag_ros
)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  walkCommand.msg
  ECJointMotordata.msg
  robotQVTau.msg
  robotArmQVVD.msg
  robotTorsoState.msg
  robotPhase.msg
  robotArmInfo.msg
  robotHandPosition.msg
  robotHeadMotionData.msg
  armTargetPoses.msg
  imuData.msg
  jointData.msg
  endEffectorData.msg
  sensorsData.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  srvChangePhases.srv  
  srvClearPositionCMD.srv
  srvchangeCtlMode.srv
  changeArmCtrlMode.srv
  changeAMBACCtrlMode.srv
  srvChangeJoller.srv
  controlEndHand.srv
  playmusic.srv
  recordmusic.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS rospy message_runtime roscpp sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(joint_state_republisher src/joint_state_republisher.cpp)
add_dependencies(joint_state_republisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_state_republisher ${catkin_LIBRARIES})

# add_executable(tag_tracker_node src/TagTracker.cpp src/PoseTransformer.cpp)
# add_dependencies(tag_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(tag_tracker_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(TARGETS joint_state_republisher tag_tracker_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS joint_state_republisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dynamic_interface_biped.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# Add folders to be run by python nosetests
# catkin_add_nosetests(test)