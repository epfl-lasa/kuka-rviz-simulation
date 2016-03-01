#ifndef IAI_ROS_CONTROLLERS_WATCHDOG
#define IAI_ROS_CONTROLLERS_WATCHDOG

#include <ros/ros.h>

namespace iai_ros_controllers
{
  class Watchdog
  {
    public:
      Watchdog(const ros::Duration& period = ros::Duration(0.0), 
               const ros::Time& last_update = ros::Time(0.0)) 
        : period_(period), last_update_(last_update) {}
      ~Watchdog() {}

     bool isActive(const ros::Time& now) const
     {
       return now > last_update_ + period_;
     }

     void update(const ros::Time& now)
     {
       last_update_ = now;
     }

     void setPeriod(const ros::Duration& period)
     {
       period_ = period;
     }
  
    private:
      ros::Duration period_;
      ros::Time last_update_;    
  };
}
#endif
