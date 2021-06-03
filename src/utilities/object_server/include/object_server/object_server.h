#ifndef OBJECT_SERVER_H_
#define OBJECT_SERVER_H_

#include <ros/ros.h>
#include <object_server/object.h>

#include <object_msgs/Objects.h>
#include <visualization_msgs/MarkerArray.h>

#include <object_msgs/AddMovingObjects.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>

#include <interactive_markers/interactive_marker_server.h>

namespace object_server
{

  class ObjectServer
  {
  public:

    typedef std::vector<std::shared_ptr<ow::CartesianTrajectory> > Trajectories;

  public:

    ObjectServer();

    virtual ~ObjectServer();

    bool initialize(ros::NodeHandle &nh);

    void update(const ros::Time &time, const ros::Duration &dt);

  private:

    bool initalizeObjects(
      ros::NodeHandle &nh, const std::string& ns, 
      const std::string& name, Trajectories& trajectories);

    bool addTargets();

  private:

    bool addMovingObstaclesHandler(
      object_msgs::AddMovingObjectsRequest& req,
      object_msgs::AddMovingObjectsResponse& res);

    bool addMovingTargetsHandler(
      std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

    void interactiveFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  private:
    bool has_moving_targets_;

    ros::Publisher obstacles_marker_pub_;
    ros::Publisher targets_marker_pub_;
    ros::Publisher trajectories_marker_pub_;

    ros::Publisher obstacles_pub_;
    ros::Publisher targets_pub_;

    ros::ServiceServer add_moving_obstacles_server_;
    ros::ServiceServer add_moving_targets_server_;

    interactive_markers::InteractiveMarkerServer interactive_server_;
    object_msgs::Object int_obstacle_msg_;

    Trajectories obstacle_trajectories_;
    Trajectories target_trajectories_;

    std::vector<object_server::Object> targets_;
    std::vector<object_server::Object> obstacles_;

    ros::Time obstacles_start_time_;
    ros::Time targets_start_time_;
  };

} // namespace object_server

#endif