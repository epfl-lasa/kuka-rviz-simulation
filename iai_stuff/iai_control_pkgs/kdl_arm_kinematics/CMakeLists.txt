cmake_minimum_required(VERSION 2.8.3)
project(kdl_arm_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  kdl_parser
  iai_kinematics_msgs)

find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    tf
    tf_conversions
    kdl_parser
    iai_kinematics_msgs
  DEPENDS
    orocos_kdl)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS})

add_executable(arm_kinematics 
  src/arm_kinematics.cpp)
target_link_libraries(arm_kinematics 
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES})
add_dependencies(arm_kinematics ${catkin_EXPORTED_TARGETS})
