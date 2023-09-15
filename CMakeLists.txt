cmake_minimum_required(VERSION 2.8.3)
project(node_cpm_provided)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  etsi_its_msgs
  geometry_msgs
  sensor_msgs
  pdk_ros_msg
)


## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs etsi_its_msgs geometry_msgs sensor_msgs pdk_ros_msg
)

## Build the node
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(node_cpm_provided src/cpm_provided.cpp)
add_dependencies(node_cpm_provided etsi_its_msgs_generate_messages_cpp pdk_ros_msg_generate_messages_cpp )
target_link_libraries(  node_cpm_provided 
                        ${catkin_LIBRARIES}
                     )
