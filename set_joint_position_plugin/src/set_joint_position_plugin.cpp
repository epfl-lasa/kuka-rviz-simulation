/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: my_robot_plugin plugin for manipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
 */

#include <algorithm>
#include <assert.h>

#include <set_joint_position_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(set_joint_position_plugin);

////////////////////////////////////////////////////////////////////////////////
// Constructor
set_joint_position_plugin::set_joint_position_plugin()
{
  // this->wrench_msg_.force.x = 0;
  // this->wrench_msg_.force.y = 0;
  // this->wrench_msg_.force.z = 0;
  // this->wrench_msg_.torque.x = 0;
  // this->wrench_msg_.torque.y = 0;
  // this->wrench_msg_.torque.z = 0;

  //    this->twist_msg_.linear.x = 0;
  //    this->twist_msg_.linear.y = 0;
  //    this->twist_msg_.linear.z = 0;
  //    this->twist_msg_.angular.x = 0;
  //    this->twist_msg_.angular.y = 0;
  //    this->twist_msg_.angular.z = 0;

  for (int i = 0; i < 30; i++)
  {
    this->fp_torques[i] = 0.0;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
set_joint_position_plugin::~set_joint_position_plugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  this->joints_list.clear();
  this->link_name_.clear();
  this->pub_positions.shutdown();
  this->pub_positions2.shutdown();
  this->sub_.shutdown();


  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void set_joint_position_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();
  this->model_ = model_;

  // load parameters
  this->robot_namespace_ = "";

  if ( _sdf->HasElement("robotNamespace") ) this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

//  if ( !_sdf->HasElement("bodyName") )
//  {
//    ROS_FATAL("force plugin missing <bodyName>, cannot proceed");
//    return;
//  }
//  else this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

//  this->link_ = _model->GetLink(this->link_name_);

//  if (!this->link_)
//  {
//    ROS_FATAL( "set_joint_position_plugin plugin error: link named: %s does not exist\n", this->link_name_.c_str() );

//    //        return; // apparently it cannot find the link on the allegro_hand fingers, althoug it IS there...
//  }

  if ( !_sdf->HasElement("topicName") )
  {
    ROS_FATAL("force plugin missing <topicName>, cannot proceed");
    return;
  }
  else this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();


  // Make sure the ROS node for Gazebo has already been initialized
  if ( !ros::isInitialized() )
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO_STREAM("topicName:" << this->topic_name_);
  ROS_INFO_STREAM("Loading plugin for setting joint positions !:");

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);


  this->sub_ = this->rosnode_->subscribe( this->topic_name_, 1, &set_joint_position_plugin::CallBackMethod, this, ros::TransportHints().tcpNoDelay() );


  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind(&set_joint_position_plugin::QueueThread, this) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&set_joint_position_plugin::UpdateChild, this) );


  /* Checking the list of joints */
  this->joints_list = _model->GetJoints();

  // mClock.Reset();
  // mClock.SetInternal(true);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
//void set_joint_position_plugin::CallBackMethod(const std_msgs::Float32MultiArray::ConstPtr& _msg)
//void set_joint_position_plugin::CallBackMethod(const sensor_msgs::JointState::ConstPtr& _msg)
void set_joint_position_plugin::CallBackMethod(const sensor_msgs::JointState _msg)
{
  this->lock_.lock();
  // ROS_INFO_STREAM( "Getting a new position" );
  set_joint_state_ =  _msg;
  this->lock_.unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void set_joint_position_plugin::UpdateChild()
{
  this->lock_.lock();

  for (int i = 0; i < set_joint_state_.position.size();i++){
    #if GAZEBO_MAJOR_VERSION == 2
        this->joints_list[i]->SetAngle(0, set_joint_state_.position[i]);   // add the force ... // not working with gazebo 4 ?? --> function is virtual, depends on the physics engine. How to do ??
    #else // if GAZEBO_MAJOR_VERSION == 2
        this->joints_list[i]->SetPosition(0, set_joint_state_.position[i]);   // add the force ... // not working with gazebo 4 ?? --> function is virtual, depends on the physics engine. How to do ??
    #endif // if GAZEBO_MAJOR_VERSION == 2

  }





  this->lock_.unlock();

}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void set_joint_position_plugin::QueueThread()
{
  static const double timeout = 0.01;

  while ( this->rosnode_->ok() )
  {
    this->queue_.callAvailable( ros::WallDuration(timeout) );
  }
}


}
