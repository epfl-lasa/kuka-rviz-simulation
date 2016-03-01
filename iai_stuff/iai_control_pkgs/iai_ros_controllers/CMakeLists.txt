cmake_minimum_required(VERSION 2.8.3)
project(iai_ros_controllers)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  controller_interface 
  iai_hardware_interface
  realtime_tools
  iai_control_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp controller_interface iai_hardware_interface 
                 realtime_tools iai_control_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/MultiJointVelocityImpedanceController.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
