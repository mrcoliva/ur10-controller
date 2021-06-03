#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

#include <object_server/utilities.h>

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // nothing to be done
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_obstacle_node");
  ROS_INFO_STREAM("starting interactive_obstacle_node...");

  // create an interactive marker server on the topic namespace interactive_obstacle
  interactive_markers::InteractiveMarkerServer server("interactive_obstacle");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "obstacle";
  int_marker.description = "interactive_obstacle";

  geometry_msgs::Pose X_init_w;
  X_init_w.position.x = 2.0;
  X_init_w.position.y = 2.0;
  X_init_w.position.z = 2.0;
  X_init_w.orientation.x = 0.0;
  X_init_w.orientation.y = 0.0;
  X_init_w.orientation.z = 0.0;
  X_init_w.orientation.w = 1.0;

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time::now();

  // create a grey box marker
  visualization_msgs::Marker sphere_marker = object_server::create_sphere("", 0, 0.3);
  
  // create a non-interactive control which contains the shere
  visualization_msgs::InteractiveMarkerControl sphere_control;
  sphere_control.always_visible = true;
  sphere_control.markers.push_back( sphere_marker );
  int_marker.controls.push_back( sphere_control );

  // create interactive controls for each cartesian axis
  visualization_msgs::InteractiveMarkerControl x_control, y_control, z_control;

  x_control.name = "move_x";
  x_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(x_control);

  y_control.name = "move_y";
  y_control.orientation.w = 1.0; y_control.orientation.y = 1.0;
  y_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(y_control);

  z_control.name = "move_z";
  z_control.orientation.w = 1.0; z_control.orientation.z = 1.0;
  z_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(z_control);

  // add the interactive marker to our collection &
  server.insert(int_marker, &processFeedback);

  server.setPose("obstacle", X_init_w, header);

  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}