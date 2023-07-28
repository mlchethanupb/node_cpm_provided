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

set(CMAKE_PREFIX_PATH "/home/airscope/workspace/other_repos/flatbuffers/" ${CMAKE_PREFIX_PATH})
set(CMAKE_MODULE_PATH "/home/airscope/workspace/other_repos/flatbuffers/" ${CMAKE_MODULE_PATH})

# Find FlatBuffers package
find_package(Flatbuffers REQUIRED)

# Generate code from FlatBuffers schema
set(FLATBUFFERS_SOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/flatbuffers)
set(FLATBUFFERS_SCHEMA ${FLATBUFFERS_SOURCES_DIR}/all_interface.fbs)

# Add the generated FlatBuffers code as a library
set(FLATBUFFERS_GENERATED_FILES
  ${FLATBUFFERS_SOURCES_DIR}/all_interface_generated.h
)

add_library(etsi_flatbuffers_lib_2 ${FLATBUFFERS_GENERATED_FILES})
set_target_properties(etsi_flatbuffers_lib_2 PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(etsi_flatbuffers_lib_2 PUBLIC ${FLATBUFFERS_LIBRARIES})

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs
  etsi_its_msgs
  geometry_msgs
  sensor_msgs # Add other necessary dependencies
  pdk_ros_msg
)

## Build the node
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Flatbuffers_INCLUDE_DIRS}
  /home/airscope/workspace/other_repos/flatbuffers/include
)

add_executable(node_cpm_provided src/cpm_provided.cpp)
target_link_libraries(  node_cpm_provided 
                        ${catkin_LIBRARIES}
                     )
