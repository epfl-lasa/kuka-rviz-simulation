#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

class Pr2VelocityControllerDemux
{
  public:
    Pr2VelocityControllerDemux(const ros::NodeHandle& nh) :
        nh_(nh)
    {
      subscriber_ = nh_.subscribe("in_topic", 1, 
          &Pr2VelocityControllerDemux::callback, this);

      shoulder_pan_pub_ = nh_.advertise<std_msgs::Float64>("shoulder_pan", 1);
      shoulder_lift_pub_ = nh_.advertise<std_msgs::Float64>("shoulder_lift", 1);
      upper_arm_roll_pub_ = nh_.advertise<std_msgs::Float64>("upper_arm_roll", 1);
      elbow_flex_pub_ = nh_.advertise<std_msgs::Float64>("elbow_flex", 1);
      forearm_roll_pub_ = nh_.advertise<std_msgs::Float64>("forearm_roll", 1);
      wrist_flex_pub_ = nh_.advertise<std_msgs::Float64>("wrist_flex", 1);
      wrist_roll_pub_ = nh_.advertise<std_msgs::Float64>("wrist_roll", 1);
    }
 
    ~Pr2VelocityControllerDemux() {}

  private:
    // ROS communication ...
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    // ... publishers for every joint
    ros::Publisher shoulder_pan_pub_;
    ros::Publisher shoulder_lift_pub_;
    ros::Publisher upper_arm_roll_pub_;
    ros::Publisher elbow_flex_pub_;
    ros::Publisher forearm_roll_pub_;
    ros::Publisher wrist_flex_pub_;
    ros::Publisher wrist_roll_pub_;

    void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      assert(msg->data.size() == 7);
      std_msgs::Float64 out_msg;
      
      out_msg.data = msg->data[0];
      shoulder_pan_pub_.publish(out_msg);

      out_msg.data = msg->data[1];
      shoulder_lift_pub_.publish(out_msg);

      out_msg.data = msg->data[2];
      upper_arm_roll_pub_.publish(out_msg);

      out_msg.data = msg->data[3];
      elbow_flex_pub_.publish(out_msg);

      out_msg.data = msg->data[4];
      forearm_roll_pub_.publish(out_msg);

      out_msg.data = msg->data[5];
      wrist_flex_pub_.publish(out_msg);

      out_msg.data = msg->data[6];
      wrist_roll_pub_.publish(out_msg);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_velocity_controller_demux");

  ros::NodeHandle nh("~");

  Pr2VelocityControllerDemux my_demux(nh);

  ros::spin();

  return 0;
}
