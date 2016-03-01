project(iai_velocity_controller_demux)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS})

add_executable(iai_pr2_velocity_controller_demux
  src/iai_velocity_controller_demux/iai_pr2_velocity_controller_demux.cpp)

target_link_libraries(iai_pr2_velocity_controller_demux
  ${catkin_LIBRARIES})
