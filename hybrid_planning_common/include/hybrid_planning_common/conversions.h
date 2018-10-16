#ifndef HYBRID_PLANNING_CONVERSIONS_H
#define HYBRID_PLANNING_CONVERSIONS_H

#include <trajectory_msgs/JointTrajectory.h>
#include <trajopt/common.hpp>

namespace hybrid_planning_common
{

namespace descartes
{

/**
 * @brief Convert a "flat" Descartes (light) solution to a joint trajectory msg
 */
trajectory_msgs::JointTrajectory toJointTrajectory(const std::vector<double>& flat_solution,
                                                   const std::vector<std::string>& joint_names,
                                                   const ros::Duration& time_per_pt = ros::Duration(1.0));

} // descartes ns

namespace trajopt
{

/**
 * @brief Convert a "TrajArray" (from Trajopt) solution to a joint trajectory msg
 */
trajectory_msgs::JointTrajectory toJointTrajectory(const tesseract::TrajArray& traj_array,
                                                   const std::vector<std::string>& joint_names,
                                                   const ros::Duration& time_per_pt = ros::Duration(1.0));

} // trajopt ns

} // hybrid_planning_common ns

#endif // HYBRID_PLANNING_CONVERSIONS_H
