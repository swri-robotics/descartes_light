#ifndef HYBRID_PLANNING_PATH_VISUALIZATION_H
#define HYBRID_PLANNING_PATH_VISUALIZATION_H

#include <tesseract_ros/kdl/kdl_env.h>
#include <geometry_msgs/PoseArray.h>
#include "hybrid_planning_common/eigen_typedefs.h"

#include <ros/ros.h>

namespace hybrid_planning_common
{

std::vector<geometry_msgs::Pose> toPoses(const AlignedVector<Eigen::Isometry3d>& vs);

geometry_msgs::PoseArray toPoseArray(const AlignedVector<Eigen::Isometry3d>& vs,
                                     const std::string& ref_frame);

// For publishing the path async. during the synchronous exec of the examples:
template<class Msg>
class Republisher
{
public:
  Republisher(const std::string& topic, const Msg& msg, const ros::Rate& rate = ros::Rate(1))
    : msg_(msg)
  {
    ros::NodeHandle nh;
    pub_ = nh.advertise<Msg>(topic, 1);
    timer_ = nh.createTimer(rate, &Republisher::publish, this);
  }

  void publish(const ros::TimerEvent&)
  {
    pub_.publish(msg_);
  }

private:
  Msg msg_;
  ros::Publisher pub_;
  ros::Timer timer_;
};

}

#endif
