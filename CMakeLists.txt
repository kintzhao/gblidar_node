cmake_minimum_required(VERSION 2.8.3)
project(gblidar_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIB_DIRS} )

add_executable(gblidar_node src/gblidar_node.cpp  src/GBlidarDriver.cpp)
target_link_libraries(gblidar_node
    ${catkin_LIBRARIES})

