cmake_minimum_required(VERSION 2.8.3)
project(iai_control_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation std_msgs geometry_msgs actionlib_msgs
    )

add_message_files(
  FILES
  MultiJointVelocityCommand.msg
  MultiJointVelocityImpedanceCommand.msg
  MultiJointVelocityImpedanceState.msg
  CartGoal.msg
  CartState.msg
  JointLimits.msg)

add_action_files(
  DIRECTORY action
  FILES PTU.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs actionlib_msgs)
