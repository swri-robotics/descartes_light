#ifndef HYBRID_PLANNING_PATH_TYPES_H
#define HYBRID_PLANNING_PATH_TYPES_H

#include "hybrid_planning_common/eigen_typedefs.h"

namespace hybrid_planning_common
{

// Typedefs for the paths
using Pass = hybrid_planning_common::AlignedVector<Eigen::Isometry3d>;
using Path = std::vector<Pass>;

/**
 * @brief Turn a vector of vector of poses into a vector of poses by concatenating each sub vec end to end
 */
hybrid_planning_common::AlignedVector<Eigen::Isometry3d> flatten(const Path& path);

}

#endif // HYBRID_PLANNING_PATH_TYPES_H
