#include <object_server/object_server.h>
#include <ow_core/common/ros.h>

namespace object_server
{

ObjectServer::ObjectServer()
  : has_moving_targets_(false),
    interactive_server_("interactive_server")
{
}

ObjectServer::~ObjectServer()
{
}

bool ObjectServer::initialize(ros::NodeHandle& nh)
{
  // ros connections
  obstacles_marker_pub_  = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_marker", 1);
  targets_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/target_markers", 1);
  trajectories_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1);

  targets_pub_ = nh.advertise<object_msgs::Objects>("targets", 1);
  obstacles_pub_ = nh.advertise<object_msgs::Objects>("/obstacles", 1);

  add_moving_obstacles_server_ = nh.advertiseService("/add_moving_obstacles", &ObjectServer::addMovingObstaclesHandler, this);
  add_moving_targets_server_ = nh.advertiseService("/add_moving_targets", &ObjectServer::addMovingTargetsHandler, this);
  
  // load trajectories
  std::string ns = "/object_server_node/";

  int obstacle_num;
  if (!ow::load(ns + "description/obstacle_num", obstacle_num))
    return false;
  
  int target_num;
  if (!ow::load(ns + "description/target_num", target_num))
    return false;

  obstacle_trajectories_.resize(obstacle_num);
  if (!initalizeObjects(nh, ns, "obj_", obstacle_trajectories_))
    return false;

  target_trajectories_.resize(target_num);
  if (!initalizeObjects(nh, ns, "target_", target_trajectories_))
    return false;

  obstacles_start_time_ = ros::Time::now();
  targets_start_time_ = ros::Time::now();
  addTargets();

  // setup one interactive obstacle
  visualization_msgs::InteractiveMarker int_marker = create_interactive_marker(object_server::create_sphere(ns, 0, 0.3));
  int_marker.header.frame_id = "map";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "interactive_obstacle";
  int_marker.description = "interactive_obstacle";
  interactive_server_.insert(int_marker, boost::bind(&ObjectServer::interactiveFeedback, this, _1));

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time::now();

  int_obstacle_msg_.position.position.x = 2.0;
  int_obstacle_msg_.position.position.y = 2.0;
  int_obstacle_msg_.position.position.z = 2.0;
  int_obstacle_msg_.position.orientation.w = 1.0;
  interactive_server_.setPose("interactive_obstacle", int_obstacle_msg_.position, header);
  interactive_server_.applyChanges();

  return true;
}

bool ObjectServer::initalizeObjects(
  ros::NodeHandle &nh, const std::string& ns, const std::string& name, Trajectories& trajectories)
{
  ow::Scalar period;
  ow::LinearPosition x0_map;
  ow::LinearPosition n_map; 
  ow::LinearPosition d_map;
  for(size_t i = 0; i < trajectories.size(); ++i)
  {
    ow::Scalar r_fac = 1.0;
    std::vector<ow::Scalar> param;
    std::string id = name + std::to_string(i);

    if (!ow::load(ns + "description/" + id + "/period", period))
      return false;
    if (!ow::load(ns + "description/" + id + "/x0_map", x0_map))
      return false;
    if (!ow::load(ns + "description/" + id + "/n_map", n_map))
      return false;
    if (!ow::load(ns + "description/" + id + "/d_map", d_map))
      return false;
    ow::load(ns + "description/" + id + "/r_fac", r_fac);
    ow::load(ns + "description/" + id + "/param", param);

    if(param.size() == 3)
    {
      trajectories[i] =
        std::make_shared<ow::ParametricPlanarCartesianTrajectory>
          (period, x0_map, n_map, x0_map + d_map, param[0], param[1], param[2]);
    }
    else if(param.size() == 4)
    {
      trajectories[i] =
        std::make_shared<ow::ParametricCartesianTrajectory>
          (period, x0_map, r_fac, param[0], param[1], param[2]);
    }
    else
    {
      trajectories[i] =
        std::make_shared<ow::CircularCartesianTrajectory>
          (period, x0_map, n_map, x0_map + d_map, r_fac);
    }
  }
  return true;
}

void ObjectServer::update(const ros::Time& time, const ros::Duration& dt)
{
  // move and publish everything
  ros::Duration elapsed;

  // obstacles
  visualization_msgs::MarkerArray obstacle_marker_msg;
  object_msgs::Objects obstacles_msg;

  obstacle_marker_msg.markers.resize(obstacles_.size());
  obstacles_msg.objects.resize(obstacles_.size());
  elapsed = time - obstacles_start_time_;
  for(size_t i = 0; i < obstacles_.size(); ++i)
  {
    obstacles_[i].update(time, elapsed);
    obstacle_marker_msg.markers[i] = obstacles_[i].toMarkerMsg();
    obstacles_msg.objects[i] = obstacles_[i].toObjectMsg();
  }
  obstacles_msg.objects.push_back(int_obstacle_msg_);

  // targets
  visualization_msgs::MarkerArray target_marker_msg;
  object_msgs::Objects target_msg;

  elapsed = time - targets_start_time_; 
  target_marker_msg.markers.resize(targets_.size());
  target_msg.objects.resize(targets_.size());
  for(size_t i = 0; i < targets_.size(); ++i)
  {
    if(has_moving_targets_)
    {
      targets_[i].update(time, elapsed);
    }
    target_marker_msg.markers[i] = targets_[i].toMarkerMsg();
    target_msg.objects[i] = targets_[i].toObjectMsg();
  }

  // publish
  obstacles_pub_.publish(obstacles_msg);
  obstacles_marker_pub_.publish(obstacle_marker_msg);
  targets_pub_.publish(target_msg);
  targets_marker_pub_.publish(target_marker_msg);
}

bool ObjectServer::addMovingObstaclesHandler(
  object_msgs::AddMovingObjectsRequest& req,
  object_msgs::AddMovingObjectsResponse& res)
{
      // if (time.tD() > m_totalTime) {
      //   geometry_msgs::PoseStamped pose;
      //   pose.header.frame_id = "world";
      //   pose.header.stamp = ros::Time::now();
      //   // double tt = t/100.0 * m_circularTrajectoryPeriod;
      //   Vector3d position = applyTransformation(m_robotModel.T_B_0(), circularTrajectory(time.tD()).col(0));
      //   Quaterniond orienation = Quaterniond::Identity();
      //   pose.pose.position.x = position.x();
      //   pose.pose.position.y = position.y();
      //   pose.pose.position.z = position.z();
      //   pose.pose.orientation.x = orienation.x();
      //   pose.pose.orientation.y = orienation.y();
      //   pose.pose.orientation.z = orienation.z();
      //   pose.pose.orientation.w = orienation.w();
      //   m_circlePath.poses.push_back(pose);
      // }
  size_t num = std::min(size_t(req.num), obstacle_trajectories_.size());
  ROS_WARN_STREAM("Adding n=" << num << " obstacles to the scene");

  double radius = 0.3;
  std::string ns = "obstacles";

  visualization_msgs::MarkerArray marker_msg;
  marker_msg.markers.resize(num);

  // create obstacles
  obstacles_.resize(num);
  for(size_t i = 0; i < num; ++i)
  {
    obstacles_[i].initialize(
      i,
      "map",
      object_server::create_sphere(ns, i, radius),
      obstacle_trajectories_[i]);
    marker_msg.markers[i] = obstacles_[i].toLineMarkerMsg(100);
  }

  // publish trajecory
  obstacles_start_time_ = ros::Time::now();
  trajectories_marker_pub_.publish(marker_msg);
  return true;
}

bool ObjectServer::addTargets()
{
  size_t num = std::min(target_trajectories_.size(), size_t(5));
  ROS_WARN_STREAM("Adding n=" << num << " targets to the scene");

  std::vector<ow::Vector3> colors(5);
  colors[0] << 1,0,0;
  colors[1] << 0,1,0;
  colors[2] << 0,0,1;
  colors[3] << 1,1,0;
  colors[4] << 0,1,1;

  double radius = 0.05;
  std::string ns = "targets";

  // create obstacles
  targets_.resize(num);
  for(size_t i = 0; i < num; ++i)
  {
    targets_[i].initialize(
      10+i,
      "map",
      object_server::create_circle(ns, i, radius, colors[i].x(), colors[i].y(), colors[i].z()),
      target_trajectories_[i]);
    targets_[i].update(ros::Time::now(), ros::Duration(0.0));
  }
  return true;
}

bool ObjectServer::addMovingTargetsHandler(
  std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  visualization_msgs::MarkerArray marker_msg;
  marker_msg.markers.resize(targets_.size());

  for(size_t i = 0; i < targets_.size(); ++i)
  {
    marker_msg.markers[i] = targets_[i].toLineMarkerMsg(100);
  }
  trajectories_marker_pub_.publish(marker_msg);

  has_moving_targets_ = true;
  targets_start_time_ = ros::Time::now();
  return true;
}

void ObjectServer::interactiveFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  int_obstacle_msg_.position = feedback->pose;
}

}