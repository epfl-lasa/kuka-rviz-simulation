cmake_minimum_required(VERSION 2.8.3)
project(iai_robot_mechanism_controllers)

find_package(catkin REQUIRED COMPONENTS 
  pluginlib
  roscpp
  iai_control_msgs
  pr2_controller_interface
  pr2_mechanism_model
  pr2_mechanism_controllers
  pr2_mechanism_msgs
  nav_msgs
  filters
  control_toolbox
  realtime_tools
  pr2_controller_manager)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS iai_control_msgs nav_msgs pr2_mechanism_msgs
#  DEPENDS Boost TODO
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})
 
add_library(${PROJECT_NAME}
  src/rosie_odometry.cpp
  src/multi_joint_velocity_controller.cpp
  src/multi_joint_velocity_impedance_controller.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# TODO (install targets)
