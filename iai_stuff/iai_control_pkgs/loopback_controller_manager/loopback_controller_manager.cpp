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


#include <math.h>
#include <unistd.h>
#include <set>

#include <boost/thread.hpp>

#include "pr2_hardware_interface/hardware_interface.h"
#include "pr2_controller_manager/controller_manager.h"
#include "pr2_mechanism_model/robot.h"
#include "tinyxml.h"

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <ros/ros.h>

#include <urdf/model.h>

class LoopbackControllerManager
{
public:
  LoopbackControllerManager();
  virtual ~LoopbackControllerManager();

  // Inherited from gazebo::Controller
  void init();
  void update();
  void fini();

  void run();
private:

  pr2_hardware_interface::HardwareInterface hw_;
  pr2_controller_manager::ControllerManager *cm_;

  pr2_mechanism_model::RobotState *state_;

  void readUrdf();

  ros::NodeHandle* rosnode_; //!< pointer to ros node
  ros::Subscriber desired_angles_sub_;

  void ControllerManagerROSThread();
  void jointCallback(const sensor_msgs::JointState::ConstPtr &joints);
  boost::thread* ros_spinner_thread_;
  boost::mutex lock;

  double dt_; //!< simulation time step
  double damping_; //!< default joint damping
  double mass_; //!< fake link mass

  void simulateJoints();
};


using namespace std;
using namespace XmlRpc;


LoopbackControllerManager::LoopbackControllerManager()
  : hw_(), state_(NULL), dt_(0.0)
{
}


LoopbackControllerManager::~LoopbackControllerManager()
{
  delete cm_; 
  delete rosnode_;
  delete ros_spinner_thread_;
}

void LoopbackControllerManager::jointCallback(const sensor_msgs::JointState::ConstPtr &joints)
{
  boost::mutex::scoped_lock mutex(lock);

  if(joints->name.size() != joints->position.size())
  {
    ROS_ERROR("received invalid desired joints.");
    return;
  }

  for(unsigned int i=0; i < joints->name.size(); ++i)
  {
    // NOTE: JointState.position_ is declared read-only,
    //       but in a simulation this is supposedly safe to be written...
    state_->getJointState(joints->name[i])->position_ = joints->position[i];
  }

}

void LoopbackControllerManager::init()
{
  rosnode_ = new ros::NodeHandle("/");

  cm_ = new pr2_controller_manager::ControllerManager(&hw_,*rosnode_);
  readUrdf();  // read urdf, setup actuators, then setup mechanism control node

  ros::NodeHandle private_node = ros::NodeHandle("~");
  private_node.param<double>("dt", dt_, 0.01);
  private_node.param<double>("damping", damping_, 0.1);
  private_node.param<double>("mass", mass_, 0.1);

  ROS_INFO("dt: %f", dt_);
  ROS_INFO("damping: %f", damping_);
  ROS_INFO("mass: %f", mass_);

  desired_angles_sub_ = private_node.subscribe("desired_joints", 1, &LoopbackControllerManager::jointCallback, this);

  state_ = cm_->state_;

  // set all joints to zero
  for(unsigned int i = 0; i < state_->joint_states_.size(); i++)
  {
    pr2_mechanism_model::JointState &s = state_->joint_states_[i];
    s.position_ = 0.0;
    s.velocity_ = 0.0;
    s.measured_effort_ = 0.0;
    s.commanded_effort_ = 0.0;
  }

  // set some joints to different starting positions
  try
  {
    XmlRpcValue jointLists, nameList, valueList;
    if(private_node.getParam("joints", jointLists))
    {
      nameList = jointLists["name"];
      valueList = jointLists["position"];
      for (int index = 0; index < nameList.size(); index++)
      {
        pr2_mechanism_model::JointState *s =
          state_->getJointState((string) nameList[index]);
        if(s)
          s->position_ = (double) valueList[index];
      }
    }
  }
  catch(XmlRpcException e)
  {
    // syntax error, print the (pretty useless) error message
    ROS_WARN("Error parsing initial joint positions: %s", e.getMessage().c_str());
  }


  // adjust joint positions to not violate soft limits
  for(unsigned int i = 0; i < state_->joint_states_.size(); i++)
  {
    pr2_mechanism_model::JointState &s = state_->joint_states_[i];
    const boost::shared_ptr<urdf::JointSafety> safety = s.joint_->safety;

    if(safety)
    {
      double lower = safety->soft_lower_limit;
      double upper = safety->soft_upper_limit;
      if(lower != 0.0 || upper != 0.0)
      {
        double margin = (upper - lower) * 0.001;
        lower += margin;
        upper -= margin;
        double pos_old = s.position_;

        s.position_ = (s.position_ < lower) ? lower : s.position_;
        s.position_ = (s.position_ > upper) ? upper : s.position_;

        if(s.position_ != pos_old)
          ROS_INFO("adjusted joint %s to %f", s.joint_->name.c_str(), s.position_);
      }
    }
  }

  hw_.current_time_ = ros::Time::now();

  // pr2_etherCAT calls ros::spin(), we'll thread out one spinner here to mimic that
  ros_spinner_thread_ = new boost::thread( boost::bind( &LoopbackControllerManager::ControllerManagerROSThread,this ) );

}


void LoopbackControllerManager::simulateJoints()
{
  // \todo: We should use KDL to do correct dynamics

  for (unsigned int i = 0; i < state_->joint_states_.size(); ++i)
  {
    pr2_mechanism_model::JointState &s = state_->joint_states_[i];
    const boost::shared_ptr<urdf::JointDynamics> dynamics = state_->joint_states_[i].joint_->dynamics;

    double effort = s.commanded_effort_ - damping_*s.velocity_;

    double dv = effort/mass_*dt_;
    s.velocity_ += dv;

    double dx = s.velocity_*dt_;
    s.position_ += dx;
  }
}


void LoopbackControllerManager::update()
{
  boost::mutex::scoped_lock mutex(lock);

  // Copies the state from the gazebo joints into the mechanism joints.
  for (unsigned int i = 0; i < state_->joint_states_.size(); ++i)
  {
    state_->joint_states_[i].measured_effort_ = state_->joint_states_[i].commanded_effort_;
  }

  // Reverses the transmissions to propagate the joint position into the actuators.
  state_->propagateJointPositionToActuatorPosition();

  //--------------------------------------------------
  //  Runs Mechanism Control
  //--------------------------------------------------

  // \todo: necessary?
  hw_.current_time_ = ros::Time::now();

  try
  {
    cm_->update();
  }
  catch (const char* c)
  {
    if (strcmp(c,"dividebyzero")==0)
      ROS_WARN("pid controller reports divide by zero error");
    else
      ROS_WARN("unknown const char* exception: %s", c);
  }

  //--------------------------------------------------
  //  Simulate the effort commands
  //--------------------------------------------------

  // Reverses the transmissions to propagate the actuator commands into the joints.
  state_->propagateActuatorEffortToJointEffort();

  simulateJoints();
}

void LoopbackControllerManager::fini()
{
  ROS_DEBUG("calling LoopbackControllerManager::fini");

  //pr2_hardware_interface::ActuatorMap::const_iterator it;
  //for (it = hw_.actuators_.begin(); it != hw_.actuators_.end(); ++it)
  //  delete it->second; // why is this causing double free corrpution?
  cm_->~ControllerManager();
  rosnode_->shutdown();

  ros_spinner_thread_->join();
}

void LoopbackControllerManager::readUrdf()
{
  std::string urdf_string;

  ros::NodeHandle private_node("~");
  if (private_node.getParam("robot_description", urdf_string))
    ROS_INFO("Loaded robot description from local namespace '%s'.", private_node.getNamespace().c_str());
  else
    rosnode_->getParam("/robot_description", urdf_string);

  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(urdf_string.c_str()))
  {
    ROS_ERROR("Failed to load robot description");
    abort();
  }

  struct GetActuators : public TiXmlVisitor
  {
    std::set<std::string> actuators;
    virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
    {
      if (elt.Attribute("name") &&
            ( elt.ValueStr() == std::string("actuator")
           || elt.ValueStr() == std::string("rightActuator")
           || elt.ValueStr() == std::string("leftActuator") )) 
        actuators.insert(elt.Attribute("name"));
      return true;
    }
  } get_actuators;
  doc.RootElement()->Accept(&get_actuators);

  // Places the found actuators into the hardware interface.
  std::set<std::string>::iterator it;
  for (it = get_actuators.actuators.begin(); it != get_actuators.actuators.end(); ++it)
  {
    // ROS_INFO_STREAM("adding actuator " << *it);
    pr2_hardware_interface::Actuator* pr2_actuator = new pr2_hardware_interface::Actuator(*it);
    pr2_actuator->state_.is_enabled_ = true;
    hw_.addActuator(pr2_actuator);
  }

  // Setup mechanism control node
  cm_->initXml(doc.RootElement());

  for (unsigned int i = 0; i < cm_->state_->joint_states_.size(); ++i)
    cm_->state_->joint_states_[i].calibrated_ = true;
}


void LoopbackControllerManager::ControllerManagerROSThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  ros::Rate rate(0.2/dt_);

  while (rosnode_->ok())
  {
    rate.sleep();
    ros::spinOnce();
  }
}


void LoopbackControllerManager::run()
{

  ros::Rate rate(1.0/dt_);
  while(rosnode_->ok())
  {
    update();
    rate.sleep();
  }
}


int main(int argc, char *argv[])
{
  ros::init(argc,argv,"loopback_controllers");

  // bring up mechanism controllers
  LoopbackControllerManager m;

  m.init();
  m.run();
  m.fini();
}
