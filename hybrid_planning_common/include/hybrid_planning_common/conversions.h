#ifndef HYBRID_PLANNING_CONVERSIONS_H
#define HYBRID_PLANNING_CONVERSIONS_H

#include <trajectory_msgs/JointTrajectory.h>
#include <trajopt/common.hpp>

namespace hybrid_planning_common
{

/**
 * @brief Convert a "flat" Descartes (light) solution to a joint trajectory msg
 */
trajectory_msgs::JointTrajectory descartesToJointTrajectory(const std::vector<double>& flat_solution,
                                                            const std::vector<std::string>& joint_names,
                                                            const ros::Duration& time_per_pt = ros::Duration(1.0));


/**
 * @brief Convert a "TrajArray" (from Trajopt) solution to a joint trajectory msg
 */
trajectory_msgs::JointTrajectory trajoptToJointTrajectory(const trajopt::TrajArray& traj_array,
                                                          const std::vector<std::string>& joint_names,
                                                          const ros::Duration& time_per_pt = ros::Duration(1.0));

trajopt::TrajArray jointTrajectoryToTrajopt(const trajectory_msgs::JointTrajectory& traj);


} // hybrid_planning_common ns

#endif // HYBRID_PLANNING_CONVERSIONS_H
