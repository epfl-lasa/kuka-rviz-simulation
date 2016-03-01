cmake_minimum_required(VERSION 2.8.3)
project(iai_kinematics_msgs)

find_package (catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs message_generation)

add_message_files(
  FILES
  ErrorCodes.msg
  JointLimits.msg
  KDLWeights.msg
  KinematicSolverInfo.msg
  MultiDOFJointState.msg
  PositionIKRequest.msg
  RobotState.msg)

add_service_files(
  FILES
  GetKinematicSolverInfo.srv
  GetPositionFK.srv
  GetPositionIK.srv
  GetWeightedIK.srv)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs message_runtime)
