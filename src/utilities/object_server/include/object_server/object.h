#ifndef OBJECT_SERVER_OBJECT_H_
#define OBJECT_SERVER_OBJECT_H_

#include <ros/ros.h>
#include <object_server/utilities.h>
#include <object_server/trajectories.h>
#include <object_msgs/Object.h>

namespace object_server
{

  class Object
  {
  public:
    Object();

    virtual ~Object();

    bool initialize(
      int id,
      const std::string& frame,
      const visualization_msgs::Marker& marker, 
      const std::shared_ptr<ow::CartesianTrajectory>& trajectory);

    void update(const ros::Time& time, const ros::Duration &elapsed);

    object_msgs::Object toObjectMsg();

    visualization_msgs::Marker toMarkerMsg();

    std::vector<geometry_msgs::PoseStamped> toNavPath(size_t n_steps);

    visualization_msgs::Marker toLineMarkerMsg(size_t n_steps);

  protected:
    bool is_initialized_;
    int id_;
    std::string frame_;
    ros::Time time_;

    ow::CartesianState state_;
    visualization_msgs::Marker marker_;
    std::shared_ptr<ow::CartesianTrajectory> trajectory_;
  };

}

#endif