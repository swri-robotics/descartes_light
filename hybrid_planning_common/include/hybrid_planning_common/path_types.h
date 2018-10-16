#ifndef HYBRID_PLANNING_PATH_TYPES_H
#define HYBRID_PLANNING_PATH_TYPES_H

#include "hybrid_planning_common/eigen_typedefs.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace hybrid_planning_common
{

// Typedefs for the paths
using ToolPass = hybrid_planning_common::AlignedVector<Eigen::Isometry3d>;
using ToolPath = std::vector<ToolPass>;

using JointPass = trajectory_msgs::JointTrajectory;
using JointPath = std::vector<JointPass>;

/**
 * @brief Turn a vector of vector of poses into a vector of poses by concatenating each sub vec end to end
 */
hybrid_planning_common::AlignedVector<Eigen::Isometry3d> flatten(const ToolPath& path);

}

#endif // HYBRID_PLANNING_PATH_TYPES_H
