cmake_minimum_required(VERSION 2.8.3)
project(loopback_controller_manager_examples)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir
    launch
    config)
  install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
      PATTERN ".svn" EXCLUDE
      PATTERN ".git" EXCLUDE)
endforeach()
