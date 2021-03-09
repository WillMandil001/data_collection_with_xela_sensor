cmake_minimum_required(VERSION 2.8.3)
project(squirrel_sensing_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
)

set( CMAKE_VERBOSE_MAKEFILE on )

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    roslib
    rospy
    std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensing src/main.cpp
		       src/node.cpp
		       src/sensing_drivers.cpp)

target_link_libraries(sensing 
			${catkin_LIBRARIES})

install(DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
