#include "hybrid_planning_common/conversions.h"

trajectory_msgs::JointTrajectory
hybrid_planning_common::descartes::toJointTrajectory(const std::vector<double>& flat_solution,
                                                     const std::vector<std::string>& joint_names,
                                                     const ros::Duration& time_per_pt)
{
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = joint_names;

  const auto dof = joint_names.size();

  for (std::size_t i = 0; i < flat_solution.size() / dof; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions.assign(flat_solution.begin() + long(i * dof), flat_solution.begin() + long((i + 1) * dof));
    pt.time_from_start = time_per_pt * i;
    trajectory.points.push_back(pt);
  }
  return trajectory;
}


trajectory_msgs::JointTrajectory
hybrid_planning_common::trajopt::toJointTrajectory(const tesseract::TrajArray& traj_array,
                                                   const std::vector<std::string>& joint_names,
                                                   const ros::Duration& time_per_pt)
{
  trajectory_msgs::JointTrajectory out;
  out.joint_names = joint_names;
  for (int i = 0; i < traj_array.rows(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint jtp;
    for (int j = 0; j < traj_array.cols(); ++j)
    {
      jtp.positions.push_back(traj_array(i, j));
    }
    jtp.time_from_start = time_per_pt * i;
    out.points.push_back(jtp);
  }

  return out;
}
