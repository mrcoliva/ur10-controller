#include <ros/ros.h>
#include <object_server/object_server.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_server_node");
  ROS_INFO_STREAM("starting object_server_node...");

  ros::NodeHandle nh("~");
  object_server::ObjectServer server;

  if(!server.initialize(nh))
  {
    ROS_ERROR_STREAM("object_server_node: failed to init ObjectServer");
    return -1;
  }

  // publish at a rate of 100 Hz
  ros::Time time, prev_time, time_start;
  ros::Duration dt;
  ros::Rate rate(100);
  time_start = ros::Time::now();

  while (ros::ok())
  {
    // time update
    time = ros::Time::now();
    dt = time - prev_time;
    prev_time = time;

    server.update(time, dt);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}