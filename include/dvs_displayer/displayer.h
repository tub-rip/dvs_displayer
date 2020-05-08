#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dvs_msgs/EventArray.h>
#include <opencv2/core/core.hpp>

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
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  // Subscribers
  ros::Subscriber event_sub_;
  image_transport::Subscriber image_sub_;

  // Publishers
  image_transport::Publisher image_pub_;
  cv::Mat last_image_;
  bool used_last_image_;

  enum DisplayMethod
  {
    GRAYSCALE, RED_BLUE
  } display_method_;

  void seismic_cmap(cv::Mat& lut);
  cv::Mat cmap_; // custom colormap

};

} // namespace
