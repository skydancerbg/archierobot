cmake_minimum_required(VERSION 2.8.3)
project(archierobot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Dependencies ##

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    rospy 
    tf2
	tf2_ros
    nav_msgs 
    archie_msgs 
    geometry_msgs 
    sensor_msgs 
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES archierobot
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros nav_msgs archie_msgs geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)


## Build ##

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(archie_main src/archie_main.cpp)
add_executable(archie_main_node src/archie_main_node.cpp)
target_link_libraries(archie_main_node archie_main ${catkin_LIBRARIES})


## Install ##
# Future work


## Testing ##
# Future work
