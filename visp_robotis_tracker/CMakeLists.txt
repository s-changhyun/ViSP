cmake_minimum_required(VERSION 2.8.3)
project(visp_robotis_tracker)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs
  tf
  visp_bridge
  visp_ros
)

find_package(VISP REQUIRED)
# Admanipulationd package definitions
#add_definitions(${VISP_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
 
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_geometry
    roscpp
    sensor_msgs
    std_msgs
    visp_bridge
    
  DEPENDS
    VISP
)

###################
## Build library ##
###################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)

add_executable(visp_robotis_tracker 
  src/main.cpp
  src/node.cpp
)

target_link_libraries(visp_robotis_tracker 
  ${catkin_LIBRARIES}
)