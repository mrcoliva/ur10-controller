#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <object_msgs/Objects.h>
#include <ow_core/types.h>

#include <object_server/object.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moving_obstacle_node");
  ROS_INFO_STREAM("starting moving_obstacle_node...");

  ros::NodeHandle nh("~");
  ros::Publisher maker_pub = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_marker", 1);
  ros::Publisher trajectories_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 1);

  std::vector<object_server::Object> objects(2);
  visualization_msgs::MarkerArray marker_msg;
  visualization_msgs::MarkerArray traj_marker_msg;

  double radius = 0.3;
  std::string ns = "obstacles";

  ow::Scalar period;
  ow::LinearPosition x_init_map;
  ow::LinearPosition x;

  // publish at a rate of 100 Hz
  ros::Time time, prev_time, time_start;
  ros::Duration dt, elapsed;
  ros::Rate rate(100);
  time_start = ros::Time::now();

  marker_msg.markers.resize(1);
  marker_msg.markers[0] = object_server::create_sphere(ns, 0, radius);
  marker_msg.markers[0].header.frame_id = "map";
  ow::Scalar omega = 2*M_PI/30;

  traj_marker_msg.markers.resize(1);
  traj_marker_msg.markers[0] = object_server::create_line_strip(ns, 0);
  traj_marker_msg.markers[0].header.frame_id = "map";

  std::vector<geometry_msgs::Point> points;
  nav_msgs::Path path;
  path.header.frame_id = "map";

  ow::Vector3 p_vec, t_vec, c_vec, y_vec;
  ow::Rotation3 R = ow::Rotation3::Identity();
  ow::CartesianPosition X;

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "map";


  ow::Scalar a = 5;

  double dt_eval = 1./100.; //2 * M_PI / (omega * ow::Scalar(100));
  for (size_t i = 0; i < 100; ++i)
  {
    ow::Scalar t = i*dt_eval;
    ow::Scalar s = -2.0 * t * t * t + 3.0 * t * t;
    s = 2*s - 1;

    ROS_WARN_STREAM("t=" << t);

    /*p_vec.x() = 0.15*(sin(omega*t) + 2*sin(2*omega*t));
    p_vec.y() = 0.15*(cos(omega*t) - 2*cos(2*omega*t));
    p_vec.z() = -0.3*sin(3*omega*t) + 1;

    t_vec.x() = 0.15*omega*(cos(omega*t) + 4*cos(2*omega*t));
    t_vec.y() = -0.15*omega*(sin(omega*t) - 4*sin(2*omega*t));
    t_vec.z() = -0.3*3*cos(3*omega*t);

    c_vec.x() = -0.15*omega*omega*(sin(omega*t) + 8*sin(2*omega*t));
    c_vec.y() = 0.15*omega*omega*(cos(omega*t) - 8*cos(2*omega*t));
    c_vec.z() = 0.3*9*omega*omega*sin(3*omega*t);*/

    p_vec.x() = 0.25*sqrt(1.0 - s*s)*cos(a*M_PI*s);
    p_vec.y() = 0.25*sqrt(1.0 - s*s)*sin(a*M_PI*s);
    p_vec.z() = 0.25*s;

    t_vec.x() = -(t/sqrt(1.0 - s*s)*cos(a*M_PI*s) + sqrt(1.0 - s*s)*sin(a*M_PI*t));
    t_vec.y() = -(t/sqrt(1.0 - s*s)*sin(a*M_PI*s) - sqrt(1.0 - s*s)*cos(a*M_PI*t));
    t_vec.z() = 0;

    c_vec.x() = -0.15*omega*omega*cos(omega*t);
    c_vec.y() = -0.15*omega*omega*sin(omega*t);
    c_vec.z() = 0;

    //y_vec = t_vec.cross(c_vec);
    //R << c_vec.normalized(), y_vec.normalized(), t_vec.normalized();

    X.position() = p_vec;
    X.orientation() = R;

    points.push_back(p_vec.toPointMsg());

    pose_stamped.pose = X;
    path.poses.push_back(pose_stamped);
  }
  traj_marker_msg.markers[0].points = points;

  while (ros::ok())
  {
    // time update
    time = ros::Time::now();
    dt = time - prev_time;
    prev_time = time;
    elapsed = time - time_start;

    ow::Scalar t = elapsed.toSec();
    x.x() = 0.15*(sin(omega*t) + 2*sin(2*omega*t));
    x.y() = 0.15*(cos(omega*t) - 2*cos(2*omega*t));
    x.z() = 0.3*-sin(3*omega*t) + 1;

    marker_msg.markers[0].header.stamp = time;
    marker_msg.markers[0].pose.position = x;
    maker_pub.publish(marker_msg);

    traj_marker_msg.markers[0].header.stamp = time;
    trajectories_marker_pub.publish(traj_marker_msg);

    path_pub.publish(path);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}