cmake_minimum_required(VERSION 2.8.3)
project(loopback_controller_manager)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  urdf 
  pr2_hardware_interface
  pr2_controller_manager
  pr2_mechanism_model
  nav_msgs
  tf)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(DEPENDS Boost)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} loopback_controller_manager.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES})

add_executable(odometry_simulation odometry_simulation.cpp)
target_link_libraries(odometry_simulation ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
