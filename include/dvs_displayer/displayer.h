#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dvs_msgs/EventArray.h>

namespace dvs_displayer
{

class Displayer {
public:
  Displayer(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  virtual ~Displayer();

private:
  ros::NodeHandle nh_;

  // Callback functions
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);

  // Subscribers
  ros::Subscriber event_sub_;

  // Publishers
  image_transport::Publisher image_pub_;
};

} // namespace
