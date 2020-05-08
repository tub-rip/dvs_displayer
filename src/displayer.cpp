#include "dvs_displayer/displayer.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

namespace dvs_displayer {

Displayer::Displayer(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh)
{
  // Get parameters of display method
  std::string display_method_str;
  nh_private.param<std::string>("display_method", display_method_str, "");
  if (display_method_str == std::string("grayscale"))
  {
      display_method_ = GRAYSCALE;
  } else
  {
      display_method_ = RED_BLUE;
  }

  // Set up subscribers and publishers
  event_sub_ = nh_.subscribe("events", 1, &Displayer::eventsCallback, this);

  image_transport::ImageTransport it_(nh_);
  image_pub_ = it_.advertise("event_image", 1);
}


Displayer::~Displayer()
{
  image_pub_.shutdown();
}


void Displayer::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{

  // Exit if there are no subscribers
  if (image_pub_.getNumSubscribers() <= 0)
  {
    return;
  }

  // Create image if at least one subscriber
  cv_bridge::CvImage cv_image;
  if (msg->events.size() > 0)
  {
    cv_image.header.stamp = msg->events[msg->events.size()/2].ts;
  }

  if (display_method_ == GRAYSCALE)
  {
    // Create image
    cv_image.encoding = "mono8";
    cv_image.image = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(128));

    // Per-pixel event histograms
    cv::Mat on_events  = cv::Mat::zeros(msg->height, msg->width, CV_8U);
    cv::Mat off_events = cv::Mat::zeros(msg->height, msg->width, CV_8U);
    for(const dvs_msgs::Event& ev : msg->events)
    {
      if (ev.polarity == true)
        on_events.at<uint8_t>(ev.y, ev.x)++;
      else
        off_events.at<uint8_t>(ev.y, ev.x)++;
    }

    // Scale image to use the full range [0,255]
    // (using the same scaling factor for ON and OFF events)
    double max_on, max_off, dummy;
    cv::minMaxLoc(on_events, &dummy, &max_on);
    cv::minMaxLoc(off_events, &dummy, &max_off);
    const double max_abs_val = std::max(max_on, max_off);
    const double scale = 127 / max_abs_val;
    cv_image.image += scale * on_events;
    cv_image.image -= scale * off_events;

  }
  else if (display_method_ == RED_BLUE)
  {
    // Create image
    cv_image.encoding = "bgr8";
    cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3, cv::Scalar(255,255,255));

    // Just red or blue over white background
    for(const dvs_msgs::Event& ev : msg->events)
    {
      cv_image.image.at<cv::Vec3b>(ev.y, ev.x) =
      (ev.polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
    }

  }
  else
  {
    std::cout << "Non-implemented display method" << std::endl;
  }

  image_pub_.publish(cv_image.toImageMsg());
}

} // namespace
