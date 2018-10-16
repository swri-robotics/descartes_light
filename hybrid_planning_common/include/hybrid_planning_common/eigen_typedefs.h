#ifndef HYBRID_PLANNING_EIGEN_TYPEDEFS_H
#define HYBRID_PLANNING_EIGEN_TYPEDEFS_H

#include <vector>
#include <Eigen/Geometry>

namespace hybrid_planning_common
{

template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

}

#endif // HYBRID_PLANNING_EIGEN_TYPEDEFS_H
