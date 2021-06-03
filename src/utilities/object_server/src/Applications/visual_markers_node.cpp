#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <ow_core/types.h>

#include <object_server/utilities.h>
#include <object_server/trajectories.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_markers_node");
  ROS_INFO_STREAM("starting visual_markers_node...");

  ros::NodeHandle nh("~");
  ros::Publisher maker_pub = nh.advertise<visualization_msgs::MarkerArray>("/circles", 1);
  ros::Publisher trajectories_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1);

  visualization_msgs::MarkerArray markers;
  double radius = 0.05;
  std::string ns = "objects";

  // create markers
  markers.markers.push_back(object_server::create_circle(ns, 0, radius, 1.0, 0.0, 0.0));
  markers.markers.push_back(object_server::create_circle(ns, 1, radius, 0.0, 1.0, 0.0));
  markers.markers.push_back(object_server::create_circle(ns, 2, radius, 0.0, 0.0, 1.0));
  markers.markers.push_back(object_server::create_circle(ns, 3, radius, 1.0, 1.0, 0.0));

  // set rel. transformation
  std::vector<ow::CartesianPosition> poses_rel;
  poses_rel.push_back(ow::CartesianPosition(-0.1, -0.1, 0.0, 1.0, 0.0, 0.0, 0.0));
  poses_rel.push_back(ow::CartesianPosition(0.1, -0.1, 0.0, 1.0, 0.0, 0.0, 0.0));
  poses_rel.push_back(ow::CartesianPosition(0.1, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0));
  poses_rel.push_back(ow::CartesianPosition(-0.1, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0));

  // set absolute transformation
  ow::Scalar period = 5;
  ow::LinearPosition x_init_map(-0.6, 0.0, 0.0);
  ow::LinearPosition n_map(0, 0, 1); 
  ow::LinearPosition d_map = x_init_map + ow::LinearPosition(0.1, 0.0, 0.0);
  //ow::CircularCartesianTrajectory traj(period, x_init_map, n_map, d_map);

  ow::Scalar a = 1;
  ow::Scalar b = 2;
  ow::Scalar c = 1;
  ow::ParametricPlanarCartesianTrajectory traj(period, x_init_map, n_map, d_map, a, b, c);

  ow::CartesianPosition X_o_m(-0.6, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

  visualization_msgs::MarkerArray marker_msg;
  marker_msg.markers.resize(1);
  marker_msg.markers[0] = object_server::create_line_strip("traj", 0);
  marker_msg.markers[0].header.frame_id = "map";
  marker_msg.markers[0].header.stamp = ros::Time::now();
  marker_msg.markers[0].points = traj.toPoints(100);

  // publish at a rate of 100 Hz
  ros::Time time, prev_time, time_start;
  ros::Duration dt, elapsed;
  ros::Rate rate(100);
  time_start = ros::Time::now();
  while (ros::ok())
  {
    // time update
    time = ros::Time::now();
    dt = time - prev_time;
    prev_time = time;
    elapsed = time - time_start;

    // publish
    ow::CartesianPosition X_oi_m;
    for(size_t i = 0; i < markers.markers.size(); ++i)
    {
      X_o_m.linear() = traj.evaluate(elapsed.toSec()).pos().linear();

      X_oi_m = X_o_m*poses_rel[i];
      markers.markers[i].header.frame_id = "map";
      markers.markers[i].header.stamp = time;
      markers.markers[i].pose = X_oi_m;
    }
    maker_pub.publish(markers);
    trajectories_marker_pub_.publish(marker_msg);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}