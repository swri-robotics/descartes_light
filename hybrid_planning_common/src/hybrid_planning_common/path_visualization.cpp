#include "hybrid_planning_common/path_visualization.h"
#include <eigen_conversions/eigen_msg.h>

std::vector<geometry_msgs::Pose> hybrid_planning_common::toPoses(const AlignedVector<Eigen::Isometry3d>& vs)
{
  // Helper
  const auto to_pose_msg = [] (const Eigen::Isometry3d& ep) -> geometry_msgs::Pose {
    geometry_msgs::Pose tmp;
    tf::poseEigenToMsg(ep, tmp);
    return tmp;
  };

  std::vector<geometry_msgs::Pose> pose_msgs (vs.size());
  std::transform(begin(vs), end(vs), begin(pose_msgs), to_pose_msg);
  return pose_msgs;
}

geometry_msgs::PoseArray hybrid_planning_common::toPoseArray(const AlignedVector<Eigen::Isometry3d>& vs,
                                                             const std::string& ref_frame)
{
  geometry_msgs::PoseArray result;
  result.header.frame_id = ref_frame;
  result.header.stamp = ros::Time::now();
  result.poses = toPoses(vs);
  return result;
}
