/*
 * Copyright (c) 2014, Georg Bartels, georg.bartels@cs.uni-bremen.de
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies, 
 * either expressed or implied, of the FreeBSD Project.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

double calcDeltaX(double theta, double xdot, double ydot, double dt)
{
  return (xdot * cos(theta) - ydot * sin(theta)) * dt;
}

double calcDeltaY(double theta, double xdot, double ydot, double dt)
{
  return (xdot * sin(theta) + ydot * cos(theta)) * dt;
}

double calcDeltaTheta(double thetadot, double dt)
{
  return thetadot * dt;
}

class OdometrySimulation
{
public:
  OdometrySimulation(const ros::NodeHandle& nh);
  virtual ~OdometrySimulation();

  void init();
  void update();
  void run();

private:
  // ROS communication stuff
  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber cmd_sub_;
  tf::TransformBroadcaster tf_pub_;
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::TransformStamped tf_msg_;

  // controller simulation interval
  double sim_period_;
  // time after which watchdog should stop if no new command
  ros::Duration watchdog_period_;
  // internal odom combined state
  double x_, y_, theta_;
  // commanded base twist
  double xdot_, ydot_, thetadot_;
  // internal timing
  ros::Time last_time_, current_time_, watchdog_time_;

  // callback to get in twist commands
  void commandCallback(const geometry_msgs::Twist::ConstPtr& msg);

  // internal helper methods
  void runWatchdog();
  void updateOdomState();
  void updateMsgs();
};

OdometrySimulation::OdometrySimulation(const ros::NodeHandle& nh)
  : x_(0.0), y_(0.0), theta_(0.0), sim_period_(0.0), watchdog_period_(0.0), 
    xdot_(0.0), ydot_(0.0), thetadot_(0.0), nh_(nh)
{
}

OdometrySimulation::~OdometrySimulation()
{

}

void OdometrySimulation::init()
{
  nh_.param<double>("sim_period", sim_period_, 0.02);
  double watchdog_period;
  nh_.param<double>("watchdog_period", watchdog_period, 0.1);
  watchdog_period_ = ros::Duration(watchdog_period);
  nh_.param<std::string>("odom_frame", odom_msg_.header.frame_id, "odom");
  nh_.param<std::string>("base_frame", odom_msg_.child_frame_id, "base_link");
  tf_msg_.header.frame_id = odom_msg_.header.frame_id;
  tf_msg_.child_frame_id = odom_msg_.child_frame_id;

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  cmd_sub_ = nh_.subscribe("command", 1, 
    &OdometrySimulation::commandCallback, this);

  last_time_ = ros::Time::now();
  watchdog_time_ = ros::Time::now();

  ROS_INFO("[Odometry Simulation] Init done.");
}

void OdometrySimulation::run()
{
  ROS_INFO("[Odometry Simulation] Entering loop.");

  ros::Rate rate(1.0/sim_period_);
  while(ros::ok())
  {
    update();
    rate.sleep();
  }

  ROS_INFO("[Odometry Simulation] Leaving loop.");
}

void OdometrySimulation::update()
{
  ros::spinOnce();

  current_time_ = ros::Time::now();

  runWatchdog();
  updateOdomState();
  updateMsgs();

  odom_pub_.publish(odom_msg_);
  tf_pub_.sendTransform(tf_msg_);

  last_time_ = current_time_;
}

void OdometrySimulation::commandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  xdot_ = msg->linear.x;
  ydot_ = msg->linear.y;
  thetadot_ = msg->angular.z;

  watchdog_time_ = ros::Time::now();
}

void OdometrySimulation::runWatchdog()
{
  if ((current_time_ - watchdog_time_) > watchdog_period_) 
    xdot_ = ydot_ = thetadot_ = 0.0;
}
void OdometrySimulation::updateOdomState()
{
  double dt = (current_time_ - last_time_).toSec();

  x_ += calcDeltaX(theta_, xdot_, ydot_, dt);
  y_ += calcDeltaY(theta_, xdot_, ydot_, dt);
  theta_ += calcDeltaTheta(thetadot_, dt);
}

void OdometrySimulation::updateMsgs()
{
  geometry_msgs::Quaternion quat = 
    tf::createQuaternionMsgFromYaw(theta_);

  odom_msg_.header.stamp = current_time_;
  odom_msg_.pose.pose.position.x = x_;
  odom_msg_.pose.pose.position.y = y_;
  odom_msg_.pose.pose.position.z = 0.0;
  odom_msg_.pose.pose.orientation = quat;

  odom_msg_.twist.twist.linear.x = xdot_;
  odom_msg_.twist.twist.linear.y = ydot_;
  odom_msg_.twist.twist.angular.z = thetadot_;

  tf_msg_.header.stamp = current_time_;
  tf_msg_.transform.translation.x = x_;
  tf_msg_.transform.translation.y = y_;
  tf_msg_.transform.translation.z = 0.0;
  tf_msg_.transform.rotation = quat;
}

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"odometry_simulation");

  ros::NodeHandle nh("~");

  OdometrySimulation sim(nh);

  sim.init();
  sim.run();
}
