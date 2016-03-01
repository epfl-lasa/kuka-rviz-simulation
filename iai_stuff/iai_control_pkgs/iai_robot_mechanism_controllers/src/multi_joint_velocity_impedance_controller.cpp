/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Stuart Glaser
 */

#include "iai_robot_mechanism_controllers/multi_joint_velocity_impedance_controller.h"
#include <sstream>
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( controller::MultiJointVelocityImpedanceController, pr2_controller_interface::Controller)

namespace controller {


MultiJointVelocityImpedanceController::MultiJointVelocityImpedanceController()
  : loop_count_(0), robot_(NULL), watchdog_period(0.1)
{
}

MultiJointVelocityImpedanceController::~MultiJointVelocityImpedanceController()
{
  sub_command_.shutdown();
}

bool MultiJointVelocityImpedanceController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  using namespace XmlRpc;
  node_ = n;
  robot_ = robot;

  // Gets all of the joints
  XmlRpc::XmlRpcValue joint_names;
  if (!node_.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < joint_names.size(); ++i)
  {
    XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
                node_.getNamespace().c_str());
      return false;
    }

    pr2_mechanism_model::JointState *j = robot->getJointState((std::string)name_value);
    if (!j) {
      ROS_ERROR("Joint not found: %s. (namespace: %s)",
                ((std::string)name_value).c_str(), node_.getNamespace().c_str());
      return false;
    }
    joints_.push_back(j);
  }

  // Ensures that all the joints are calibrated.
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i]->calibrated_)
    {
      ROS_ERROR("Joint %s was not calibrated (namespace: %s)",
                joints_[i]->joint_->name.c_str(), node_.getNamespace().c_str());
      return false;
    }
  }

  // Sets up pid controllers for all of the joints
  std::string gains_ns;
  if (!node_.getParam("gains", gains_ns))
    gains_ns = node_.getNamespace() + "/gains";
  pids_.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
    if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + joints_[i]->joint_->name)))
      return false;


  output_filters_.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    std::string p = "output_filters/" + joints_[i]->joint_->name;
    if (node_.hasParam(p))
    {
      output_filters_[i].reset(new filters::FilterChain<double>("double"));
      if (!output_filters_[i]->configure(node_.resolveName(p)))
        output_filters_[i].reset();
    }
  }

  sub_command_ = node_.subscribe("command", 1, &MultiJointVelocityImpedanceController::commandCB, this);

  command_.resize(joints_.size());

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointTrajectoryControllerState>
    (node_, "state", 1));
  controller_state_publisher_->lock();
  for (size_t j = 0; j < joints_.size(); ++j)
    controller_state_publisher_->msg_.joint_names.push_back(joints_[j]->joint_->name);
  controller_state_publisher_->msg_.desired.velocities.resize(joints_.size());
  controller_state_publisher_->msg_.actual.positions.resize(joints_.size());
  controller_state_publisher_->msg_.actual.velocities.resize(joints_.size());
  controller_state_publisher_->msg_.error.velocities.resize(joints_.size());
  controller_state_publisher_->unlock();


  return true;
}

void MultiJointVelocityImpedanceController::starting()
{
  last_time_ = robot_->getTime();
  watchdog_time_ = robot_->getTime();


  //Clear the state of the PIDs
  for (size_t i = 0; i < pids_.size(); ++i)
    pids_[i].reset();

}

void MultiJointVelocityImpedanceController::update()
{
  ros::Time time = robot_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;

  double command[joints_.size()];

  boost::mutex::scoped_lock guard(command_mutex_);
  for(unsigned int i=0; i < command_.size(); ++i)
    command[i] = command_[i];
  guard.unlock();

  bool watchdog_active = ( (time - watchdog_time_) > watchdog_period );
  if (watchdog_active) {

    // set desired velocities to zero to stop joints
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      command[i] = 0.0;
    }
  }

  double error[joints_.size()];
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    error[i] = command[i] - joints_[i]->velocity_;
    double effort = pids_[i].computeCommand(error[i], dt);
    double effort_filtered = effort;
    if (output_filters_[i])
      output_filters_[i]->update(effort, effort_filtered);
    joints_[i]->commanded_effort_ += effort_filtered;
  }

  // ------ State publishing

  if (loop_count_ % 10 == 0)
  {
    if (controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        controller_state_publisher_->msg_.desired.velocities[j] = command[j];
        controller_state_publisher_->msg_.actual.positions[j] = joints_[j]->position_;
        controller_state_publisher_->msg_.actual.velocities[j] = joints_[j]->velocity_;
        controller_state_publisher_->msg_.error.velocities[j] = error[j];
      }
      controller_state_publisher_->unlockAndPublish();
    }
  }

  ++loop_count_;
}

void MultiJointVelocityImpedanceController::commandCB(
  const iai_control_msgs::MultiJointVelocityImpedanceCommand::ConstPtr &msg)
{
  boost::mutex::scoped_lock guard(command_mutex_);
  if(command_.size() == msg->velocity.size())
  {
    for(unsigned int i=0; i < command_.size(); ++i)
      command_[i] = msg->velocity[i];

      watchdog_time_ = robot_->getTime();
  }
  else
  {
    ROS_ERROR("Velocity command vector has invalid length. Expected: %lu, received: %lu.", 
      command_.size(), msg->velocity.size());
  }
}

}
