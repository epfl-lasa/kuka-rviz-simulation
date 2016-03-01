#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>
#include <vector>

class TfRelay
{
  public:
    TfRelay(const ros::NodeHandle& nh) : 
        nh_(nh)
    {
      subscriber_ = nh_.subscribe("in_topic", 1, &TfRelay::callback, this);
    }
  
    ~TfRelay() {}

  private:
    // ROS communication
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    tf::TransformBroadcaster broadcaster_;

    void callback(const tf::tfMessage::ConstPtr& msg)
    {
      // future-date the transforms to avoid lags
      std::vector<geometry_msgs::TransformStamped> transforms = msg->transforms;
      update_timestamps(transforms);
      broadcaster_.sendTransform(transforms);

//      broadcaster_.sendTransform(msg->transforms);
    }

    void update_timestamps(std::vector<geometry_msgs::TransformStamped>& transforms)
    {
      for(unsigned int i=0; i<transforms.size(); i++)
        transforms[i].header.stamp = ros::Time::now() + ros::Duration(0.01);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_relay");

  ros::NodeHandle nh("~");

  TfRelay my_relay(nh);  

  ros::spin();

  return 0;
}
