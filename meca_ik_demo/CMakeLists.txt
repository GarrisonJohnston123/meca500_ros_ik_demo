cmake_minimum_required(VERSION 3.0.2)
project(meca_ik_demo)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  sensor_msgs
  tf
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


