#include <iai_ros_controllers/MultiJointVelocityImpedanceController.h>
#include <pluginlib/class_list_macros.h>

namespace iai_ros_controllers
{
  typedef MultiJointVelocityImpedanceController MJVIC;
  typedef MultiJointVelocityImpedanceCommand MJVICommand;
  MJVIC::MultiJointVelocityImpedanceController() {} 
  MJVIC::~MultiJointVelocityImpedanceController() {}

  bool MJVIC::init(hardware_interface::EffortImpedanceJointInterface* hw, ros::NodeHandle &nh)
  {
    nh_ = nh;

    if(!nh.getParam("joints", joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam 'joints' (namespace: " << nh.getNamespace() << ").");
      return false;
    }

    if(joint_names_.size() == 0)
    {
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    if(!nh.getParam("default_stiffness", default_stiffness_))
    {
      ROS_ERROR_STREAM("Failed to getParam 'default_stiffness_' (namespace: " << nh.getNamespace() << ").");
      return false;
    }

    if(!nh.getParam("default_damping", default_damping_))
    {
      ROS_ERROR_STREAM("Failed to getParam 'default_damping_' (namespace: " << nh.getNamespace() << ").");
      return false;
    }

    double state_publish_rate;
    if(!nh.getParam("state_publish_rate", state_publish_rate))
    {
      ROS_ERROR_STREAM("Failed to getParam 'state_publish_rate' (namespace: " << nh.getNamespace() << ").");
      return false;
    }

    if(state_publish_rate <= 0.0)
    {
      ROS_ERROR_STREAM("State publish rate is not greater than 0.");
      return false;
    }

    state_publisher_period_ = ros::Duration(1.0 / state_publish_rate);

    double watchdog_period;
    if(!nh.getParam("watchdog_period", watchdog_period))
    {
      ROS_ERROR_STREAM("Failed to getParam 'watchdog_period' (namespace: " << nh.getNamespace() << ").");
      return false;
    }

    if(watchdog_period <= 0.0)
    {
      ROS_ERROR_STREAM("Watchdog period is not greater than 0.");
      return false;
    }

    watchdog_.setPeriod(ros::Duration(watchdog_period));

    for(unsigned int i=0; i<joint_names_.size(); i++)
    {
      try
      {
        joints_.push_back(hw->getHandle(joint_names_[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }

    tmp_cmd_rt_ = MJVICommand(joint_names_.size(), 0.0, 
      default_stiffness_, default_damping_);
    tmp_cmd_non_rt_ = tmp_cmd_rt_;
    cmd_buffer_.set(tmp_cmd_rt_);

    time_buffer_.set(ros::Time(0.0));

    cmd_sub_ = nh.subscribe<iai_control_msgs::MultiJointVelocityImpedanceCommand>(
      "command", 1, &MJVIC::callback, this);

    initStatePublisher(nh);

    ROS_DEBUG_STREAM(nh_.getNamespace() << ": watchdog_period: " << watchdog_period << 
                                           ", state_publish_rate: " << state_publish_rate <<
                                           ", default_damping: " << default_damping_ <<
                                           ", default_stiffness: " << default_stiffness_);

    return true;
  }

  void MJVIC::starting(const ros::Time& time)
  {
    ROS_DEBUG_STREAM(nh_.getNamespace() << ": started controller.");

    last_state_publish_time_ = time;
    time_buffer_.set(time);
    if(watchdog_.isActive(time))
      ROS_WARN_STREAM(nh_.getNamespace() << ": watchdog activated.");
  }

  void MJVIC::stopping(const ros::Time& time)
  {
    // zero commands
    for(unsigned int i=0; i<joint_names_.size(); i++)
    {
      tmp_cmd_rt_.velocities_[i] = 0.0; 
      tmp_cmd_rt_.stiffnesses_[i] = default_stiffness_; 
      tmp_cmd_rt_.dampings_[i] = default_damping_; 
    }
    cmd_buffer_.set(tmp_cmd_rt_);

    // send commands to hardware
    for(unsigned int i=0; i<joint_names_.size(); i++)
    { 
      joints_[i].setCommand(tmp_cmd_rt_.velocities_[i]); 
      joints_[i].setStiffness(tmp_cmd_rt_.stiffnesses_[i]); 
      joints_[i].setDamping(tmp_cmd_rt_.dampings_[i]); 
    }

    ROS_DEBUG_STREAM(nh_.getNamespace() << ": stopped controller.");
  }

  void MJVIC::update(const ros::Time& time, const ros::Duration& period)
  {
    time_buffer_.set(time);
    cmd_buffer_.get(tmp_cmd_rt_);

    if(watchdog_.isActive(time))
    {
      ROS_DEBUG_STREAM(nh_.getNamespace() << ": watchdog active.");

      if(!watchdog_.isActive(time-period))
        ROS_WARN_STREAM(nh_.getNamespace() << ": watchdog activated.");

      // zero commands
      for(unsigned int i=0; i<joint_names_.size(); i++)
      {
        tmp_cmd_rt_.velocities_[i] = 0.0; 
        tmp_cmd_rt_.stiffnesses_[i] = default_stiffness_; 
        tmp_cmd_rt_.dampings_[i] = default_damping_; 
      }
      cmd_buffer_.set(tmp_cmd_rt_);
    }
    
    // send commands to hardware
    for(unsigned int i=0; i<joint_names_.size(); i++)
    { 
      joints_[i].setCommand(tmp_cmd_rt_.velocities_[i]); 
      joints_[i].setStiffness(tmp_cmd_rt_.stiffnesses_[i]); 
      joints_[i].setDamping(tmp_cmd_rt_.dampings_[i]); 
    }

    // publish state
    if (!state_publisher_period_.isZero() && 
        last_state_publish_time_ + state_publisher_period_ < time)
    {
      if (state_pub_.trylock())
      {
        last_state_publish_time_ += state_publisher_period_;
        state_pub_.msg_.header.stamp = time;
        state_pub_.msg_.velocity = tmp_cmd_rt_.velocities_; 
        state_pub_.msg_.stiffness = tmp_cmd_rt_.stiffnesses_; 
        state_pub_.msg_.damping = tmp_cmd_rt_.dampings_; 
        state_pub_.unlockAndPublish();
      }
    }
  }

  void MJVIC::callback(const iai_control_msgs::MultiJointVelocityImpedanceCommandConstPtr& msg)
  {
    ROS_DEBUG_STREAM(nh_.getNamespace() << ": Received a command callback.");

    if(msg->velocity.size() != joint_names_.size())
    {
      ROS_ERROR_STREAM(nh_.getNamespace() << ": velocity command of size " <<
          msg->velocity.size() << ", expected a size of " << joint_names_.size() <<
          ". Ignoring entire command.");

      return;
    }

    if(msg->stiffness.size() != joint_names_.size())
    {
      ROS_ERROR_STREAM(nh_.getNamespace() << ": stiffness command of size " << 
          msg->stiffness.size() << ", expected a size of " << joint_names_.size() <<
          ". Ignoring entire command.");

      return;
    }

    if(msg->damping.size() != joint_names_.size())
    {
      ROS_ERROR_STREAM(nh_.getNamespace() << ": damping command of size " << 
          msg->damping.size() << ", expected a size of " << joint_names_.size() <<
          ". Ignoring entire command.");

      return;
    }

    if(msg->add_torque.size() != 0)
      ROS_WARN_STREAM(nh_.getNamespace() << ": received a non-empty add_torque" <<
          " command. Ignoring this part of the command because it is not yet implemented.");


    if(msg->velocity.size() == joint_names_.size() && 
       msg->stiffness.size() == joint_names_.size() &&
       msg->damping.size() == joint_names_.size())
    {
      ROS_DEBUG_STREAM(nh_.getNamespace() << ": received a valid command.");

      for(unsigned int i=0; i<joint_names_.size(); i++)
      {
        tmp_cmd_non_rt_.velocities_[i] = msg->velocity[i];
        tmp_cmd_non_rt_.stiffnesses_[i] = msg->stiffness[i];
        tmp_cmd_non_rt_.dampings_[i] = msg->damping[i];
      }
      cmd_buffer_.set(tmp_cmd_non_rt_);

      time_buffer_.get(tmp_now_);
      if(watchdog_.isActive(tmp_now_))
        ROS_WARN_STREAM(nh_.getNamespace() << ": watchdog deactivated.");
      watchdog_.update(tmp_now_);
    }
    else
    {
      ROS_ERROR_STREAM(nh_.getNamespace() << ": received an invalid command. " <<
          "At least one of the sub-commands does not have size " << joint_names_.size()
          << ". Ignoring entire command. velocity: " << msg->velocity.size() << 
          " stiffness: " << msg->stiffness.size() << " damping: " << msg->damping.size());
    }
  }

  void MJVIC::initStatePublisher(const ros::NodeHandle& nh)
  {
    state_pub_.init(nh, "state", 1);
    state_pub_.lock();
    state_pub_.msg_.joint_names = joint_names_;
    state_pub_.msg_.velocity.resize(joint_names_.size());
    state_pub_.msg_.stiffness.resize(joint_names_.size());
    state_pub_.msg_.damping.resize(joint_names_.size());
    state_pub_.unlock();
  }
}

PLUGINLIB_EXPORT_CLASS(iai_ros_controllers::MultiJointVelocityImpedanceController, controller_interface::ControllerBase)
