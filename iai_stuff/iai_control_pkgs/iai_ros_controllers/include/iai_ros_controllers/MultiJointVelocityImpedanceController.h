#ifndef IAI_ROS_CONTROLLERS_MULTI_JOINT_VELOCITY_IMPEDANCE_COTNROLLER
#define IAI_ROS_CONTROLLERS_MULTI_JOINT_VELOCITY_IMPEDANCE_COTNROLLER

#include <controller_interface/controller.h>
#include <iai_hardware_interface/impedance_joint_interface.h>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <iai_control_msgs/MultiJointVelocityImpedanceCommand.h>
#include <iai_control_msgs/MultiJointVelocityImpedanceState.h>
#include <iai_ros_controllers/Watchdog.h>

namespace iai_ros_controllers
{
  class MultiJointVelocityImpedanceCommand
  {
    public:
      MultiJointVelocityImpedanceCommand() {}
      MultiJointVelocityImpedanceCommand(unsigned int size, double vels, double stiffs, double damps)
        : velocities_( std::vector<double>(size, vels) ), stiffnesses_( std::vector<double>(size, stiffs) ),
          dampings_( std::vector<double>(size, damps) ) {}

      std::vector<double> velocities_, stiffnesses_, dampings_;
  };

  class MultiJointVelocityImpedanceController 
    : public controller_interface::Controller<hardware_interface::EffortImpedanceJointInterface>
  {
    public:
      MultiJointVelocityImpedanceController(); 
      ~MultiJointVelocityImpedanceController();

      bool init(hardware_interface::EffortImpedanceJointInterface* hw, ros::NodeHandle &n);
      void starting(const ros::Time& time);
      void stopping(const ros::Time& time);
      void update(const ros::Time& time, const ros::Duration& period);

    private:
      ros::NodeHandle nh_;
      ros::Subscriber cmd_sub_;

      realtime_tools::RealtimePublisher<iai_control_msgs::MultiJointVelocityImpedanceState> 
        state_pub_;
      ros::Duration state_publisher_period_;
      ros::Time last_state_publish_time_;

      std::vector<std::string> joint_names_;
      std::vector<hardware_interface::ImpedanceJointHandle> joints_;
      double default_stiffness_, default_damping_;

      realtime_tools::RealtimeBox<MultiJointVelocityImpedanceCommand> cmd_buffer_;
      realtime_tools::RealtimeBox<ros::Time> time_buffer_;
      MultiJointVelocityImpedanceCommand tmp_cmd_rt_, tmp_cmd_non_rt_;
      ros::Time tmp_now_;

      Watchdog watchdog_;

      void callback(const iai_control_msgs::MultiJointVelocityImpedanceCommandConstPtr& msg);
      void initStatePublisher(const ros::NodeHandle& nh);
  };
}

#endif
