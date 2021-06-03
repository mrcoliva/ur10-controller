#include "visual_perception/visual_perception.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_perception");
  ROS_INFO_STREAM("starting visual_perception...");

  ros::NodeHandle nh("~");
  perception::VisualPerception perception;

  if (!perception.initialize(nh))
  {
    ROS_ERROR_STREAM("obstacle_detector::ObstacleDetector(): error initalize");
    return -1;
  }

  ros::Time time, prev_time;
  ros::Duration dt;

  ros::Rate rate(100);
  while (ros::ok())
  {
    // time update
    time = ros::Time::now();
    dt = time - prev_time;
    prev_time = time;

    perception.update(time, dt);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}