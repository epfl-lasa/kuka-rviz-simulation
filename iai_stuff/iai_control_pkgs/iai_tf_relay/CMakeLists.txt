cmake_minimum_required(VERSION 2.8.3)
project(iai_tf_relay)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf)

catkin_package(
  CATKIN_DEPENDS roscpp tf)

include_directories(
  ${catkin_INCLUDE_DIRS})

add_executable(tf_relay 
  src/iai_tf_relay/tf_relay.cpp)
target_link_libraries(tf_relay
  ${catkin_LIBRARIES})
