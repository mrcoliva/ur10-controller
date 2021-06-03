#ifndef VISUAL_PERCEPTION_H_
#define VISUAL_PERVEPTION_H_

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <object_msgs/Objects.h>

namespace perception
{

class VisualPerception
{
private:
  bool has_image_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber targets_sub_;
  ros::Publisher target_pub_;
  cv::Mat img_;

  int target_id_;

public:
  VisualPerception(/* args */);
  ~VisualPerception();

  bool initialize(ros::NodeHandle& nh);

  void update(const ros::Time& time, const ros::Duration& dt);

private:
  void fitCircles(cv::Mat& img_orig);

private:
  void trackBarCallback(int state);

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void targetsCallback(const object_msgs::ObjectsConstPtr& msg);

private:
  static int tackBar;
  static void relaytrackBarCallback(int state, void* user_data);

};

}


#endif