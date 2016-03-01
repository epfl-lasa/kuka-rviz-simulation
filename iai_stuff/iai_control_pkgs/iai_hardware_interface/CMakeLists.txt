cmake_minimum_required(VERSION 2.8.3)
project(iai_hardware_interface)

find_package(catkin REQUIRED COMPONENTS roscpp hardware_interface)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS hardware_interface)
