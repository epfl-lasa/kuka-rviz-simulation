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

/*!
  \author Stuart Glaser

  \class pr2_controller_interface::JointTrajectoryActionController

*/

#ifndef MULTI_JOINT_VELOCITY_IMPEDANCE_CONTROLLER_H__
#define MULTI_JOINT_VELOCITY_IMPEDANCE_CONTROLLER_H__

#include <vector>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <ros/node_handle.h>

#include <control_toolbox/pid.h>
#include <filters/filter_chain.h>
#include <pr2_controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>

#include <iai_control_msgs/MultiJointVelocityImpedanceCommand.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>


namespace controller {


class MultiJointVelocityImpedanceController :
  public pr2_controller_interface::Controller
{
public:

  MultiJointVelocityImpedanceController();
  ~MultiJointVelocityImpedanceController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  void starting();
  void update();

private:
  int loop_count_;
  pr2_mechanism_model::RobotState *robot_;
  ros::Time last_time_;
  std::vector<pr2_mechanism_model::JointState*> joints_;
  std::vector<control_toolbox::Pid> pids_;

  std::vector<boost::shared_ptr<filters::FilterChain<double> > > output_filters_;

  ros::NodeHandle node_;

  void commandCB(const iai_control_msgs::MultiJointVelocityImpedanceCommand::ConstPtr &msg);
  ros::Subscriber sub_command_;

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      pr2_controllers_msgs::JointTrajectoryControllerState> > controller_state_publisher_;

  // ------ Mechanisms for passing the commands into realtime
  //        (double buffering for commanded velocities, and guarded copy operation)

  boost::mutex command_mutex_;
  std::vector<double> command_;
  ros::Time watchdog_time_;
  ros::Duration watchdog_period;
};

}

#endif
