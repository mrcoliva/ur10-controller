#include <object_server/object.h>

namespace object_server
{

  Object::Object() : 
    is_initialized_(false),
    trajectory_(nullptr),
    state_(ow::CartesianState::Zero())
  {
  }

  Object::~Object()
  {
  }

  bool Object::initialize(
    int id,
    const std::string& frame,
    const visualization_msgs::Marker& marker, 
    const std::shared_ptr<ow::CartesianTrajectory>& trajectory)
  {
    id_ = id;
    frame_ = frame;
    marker_ = marker;
    trajectory_ = trajectory;
    is_initialized_ = true;
    return true;
  }

  void Object::update(const ros::Time& time, const ros::Duration &elapsed)
  {
    if(!is_initialized_)
      return;

    time_ = time;
    state_ = trajectory_->evaluate(elapsed.toSec());
  }

  object_msgs::Object Object::toObjectMsg()
  {
    object_msgs::Object msg;
    msg.id = id_;
    msg.type = marker_.type;
    msg.position.position = state_.pos().linear();
    msg.velocity.linear = state_.vel().linear();
    msg.acceleration.linear = state_.acc().linear();
    return msg;
  }

  visualization_msgs::Marker Object::toMarkerMsg()
  {
    marker_.header.frame_id = "map";
    marker_.header.stamp = time_;
    marker_.pose.position = state_.pos().linear();
    return marker_;
  }

  std::vector<geometry_msgs::PoseStamped> Object::toNavPath(size_t n_steps)
  {
    return trajectory_->toNavPath(n_steps);
  }

  visualization_msgs::Marker Object::toLineMarkerMsg(size_t n_steps)
  {
    visualization_msgs::Marker marker = create_line_strip("lines", id_);
    marker.color = marker_.color;
    marker.color.a = 0.5;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.points = trajectory_->toPoints(n_steps);
    return marker;
  }

}