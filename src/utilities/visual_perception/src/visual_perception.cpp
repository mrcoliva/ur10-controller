#include "visual_perception/visual_perception.h"

namespace perception
{

  int VisualPerception::tackBar = 0;

  VisualPerception::VisualPerception() : 
    has_image_(false),
    target_id_(0)
  {
  }

  VisualPerception::~VisualPerception()
  {
  }

  bool VisualPerception::initialize(ros::NodeHandle &nh)
  {
    image_transport::ImageTransport image_transport(nh);
    image_sub_ = image_transport.subscribe("/rviz1/camera1/image", 1, &VisualPerception::imageCallback, this);
    target_pub_ = nh.advertise<object_msgs::Objects>("/target", 1);
    targets_sub_ = nh.subscribe("/object_server_node/targets", 1, &VisualPerception::targetsCallback, this);

    cv::namedWindow("camera", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("target", "camera", &tackBar, 3, VisualPerception::relaytrackBarCallback, this);
    return true;
  }

  void VisualPerception::update(const ros::Time &time, const ros::Duration &dt)
  {
    if (has_image_)
    {
      has_image_ = false;

      // fitCircles(img_);

      cv::imshow("camera", img_);
      cv::waitKey(1);
    }
  }

  void VisualPerception::fitCircles(cv::Mat& img_orig)
  {
    cv::Mat img_grey;
    cv::cvtColor(img_orig, img_grey, cv::COLOR_BGR2GRAY);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(img_grey, circles, cv::HOUGH_GRADIENT, 2, 10, 200, 30);
    std::cout << "circles=" << circles.size() << std::endl;

    cv::Vec3b color_bg = img_orig.at<cv::Vec3b>(5, 5);

    std::vector<cv::Vec3f> circles_filterd;
    for(size_t i = 0; i < circles.size(); ++i)
    {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      cv::Vec3b color = img_orig.at<cv::Vec3b>(center.x, center.y);
      if(color == color_bg)
        continue;

      circles_filterd.push_back(circles[i]);
    }

    for( size_t i = 0; i < circles_filterd.size(); i++ )
    {
         cv::Point center(cvRound(circles_filterd[i][0]), cvRound(circles_filterd[i][1]));
         int radius = cvRound(circles_filterd[i][2]);
         // draw the circle center
         circle( img_orig, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( img_orig, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }
  }

  void VisualPerception::trackBarCallback(int num)
  {
    ROS_WARN_STREAM("Got new Target=" << num);
    target_id_ = num;
  }

  void VisualPerception::relaytrackBarCallback(int state, void* data)
  {
    VisualPerception* ptr = static_cast<VisualPerception*>(data);
    ptr->trackBarCallback(tackBar);
  }

  void VisualPerception::imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      img_ = cv_ptr->image.clone();
      has_image_ = true;
    }
    catch (cv_bridge::Exception &except)
    {
      ROS_ERROR("VisualPerception::imageCallback: %s", except.what());
    }
  }

  void VisualPerception::targetsCallback(const object_msgs::ObjectsConstPtr& msg)
  {
    int id = std::min(int(msg->objects.size()), target_id_);

    object_msgs::Objects objects_msg;
    objects_msg.header = msg->header;
    objects_msg.objects.push_back(msg->objects[id]);
    target_pub_.publish(objects_msg);
  }

} // namespace perception