/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 * Desc: A dynamic controller plugin that performs generic force interface.
 * Author: John Hsu
 * Date: 24 Sept 2008
 */

#ifndef MY_ROBOT_PLUGIN_HH
#define MY_ROBOT_PLUGIN_HH

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
//#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include "gazebo/transport/transport.hh"


//#include "gazebo/physics/ode/ode_inc.h"
//#include "gazebo/physics/ode/ode_inc.h"


namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup my_robot_plugin Plugin XML Reference and Example

  \brief Ros Force Plugin.

  This is a Plugin that collects data from a ROS topic and applies wrench to a body accordingly.

  Example Usage:
  \verbatim
      <gazebo>
        <plugin filename="libMY_ROBOT_PLUGIN.so" name="MY_ROBOT_PLUGIN">
          <bodyName>box_body</bodyName>
          <topicName>box_force</topicName>
        </plugin>
      </gazebo>
  \endverbatim

\{
*/

/**
           .

*/

class set_joint_position_plugin : public ModelPlugin
{
    /// \brief Constructor
public: set_joint_position_plugin();

    /// \brief Destructor
public: virtual ~set_joint_position_plugin();

    // Documentation inherited
protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited
protected: virtual void UpdateChild();

    /// \brief call back when a Wrench message is published
    /// \param[in] _msg The Incoming ROS message representing the new force to exert.
    // private: void UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& _msg);
    //private: void CallBackMethod(const geometry_msgs::Twist::ConstPtr& _msg);
//private: void CallBackMethod(const std_msgs::Float32MultiArray::ConstPtr& _msg);
//private: void CallBackMethod(const sensor_msgs::JointState::ConstPtr& _msg);
private: void CallBackMethod(const sensor_msgs::JointState _msg);

    /// \brief The custom callback queue thread function.
private: void QueueThread();

    /// \brief A pointer to the gazebo world.
private: physics::WorldPtr world_;
private: physics::ModelPtr model_;

    /// \brief A pointer to the Link, where force is applied
private: physics::LinkPtr link_;

    /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
private: ros::NodeHandle* rosnode_;
private: ros::Subscriber sub_;
private: ros::Publisher pub_positions;
private: ros::Publisher pub_positions2;
         sensor_msgs::JointState joint_state_;

private: void PublishJointState();

    /// \brief A mutex to lock access to fields that are used in ROS message callbacks
private: boost::mutex lock_;

    /// \brief ROS Wrench topic name inputs
private: std::string topic_name_;
    /// \brief The Link this plugin is attached to, and will exert forces on.
private: std::string link_name_;

    /// \brief for setting ROS name space
private: std::string robot_namespace_;

    // Custom Callback Queue
private: ros::CallbackQueue queue_;
    /// \brief Thead object for the running callback Thread.
private: boost::thread callback_queue_thread_;
    /// \brief Container for the wrench force that this plugin exerts on the body.
    //private: geometry_msgs::Wrench wrench_msg_;
private: geometry_msgs::Twist twist_msg_;
    // Pointer to the update event connection
private: event::ConnectionPtr update_connection_;


private: std::vector<physics::JointPtr> joints_list;

    //private: std_msgs::Float32MultiArray f32ma_torques;
private: float fp_torques[30];
private: sensor_msgs::JointState set_joint_state_;

// private: Clock mClock;
private: double last_time;


};
/** \} */
/// @}
}
#endif
