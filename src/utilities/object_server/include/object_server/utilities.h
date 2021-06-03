#ifndef OBJECT_SERVER_UTILITIES_H_
#define OBJECT_SERVER_UTILITIES_H_

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>

namespace object_server
{

/**
 * @brief Create a colored circle marker with radius r
 * 
 * @param id 
 * @param r 
 * @param g 
 * @param b 
 * @return visualization_msgs::Marker 
 */
inline visualization_msgs::Marker create_circle(const std::string& ns, int id, double radius, double r=1.0, double g=0.0, double b=0.0)
{
  visualization_msgs::Marker marker;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = radius;
  marker.scale.y = radius;
  marker.scale.z = 0.01;
  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  return marker;
}

inline visualization_msgs::Marker create_sphere(const std::string& ns, int id, double radius, double r=0.5, double g=0.5, double b=0.5)
{
  visualization_msgs::Marker marker;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = radius;
  marker.scale.y = radius;
  marker.scale.z = radius;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  return marker;
}

inline visualization_msgs::Marker create_line_strip(const std::string& ns, int id, double width=0.01, double r=0.5, double g=0.5, double b=0.5)
{
  visualization_msgs::Marker marker;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.scale.x = width;
  marker.color.a = 0.7;
  return marker;
}

inline visualization_msgs::InteractiveMarker create_interactive_marker(const visualization_msgs::Marker& marker)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.description = "interactive_obstacle";

  geometry_msgs::Pose X_init_w;
  X_init_w.position.x = 2.0;
  X_init_w.position.y = 2.0;
  X_init_w.position.z = 2.0;
  X_init_w.orientation.x = 0.0;
  X_init_w.orientation.y = 0.0;
  X_init_w.orientation.z = 0.0;
  X_init_w.orientation.w = 1.0;
  
  // create a non-interactive control which contains the shere
  visualization_msgs::InteractiveMarkerControl sphere_control;
  sphere_control.always_visible = true;
  sphere_control.markers.push_back( marker );
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

  return int_marker;
}

}

#endif