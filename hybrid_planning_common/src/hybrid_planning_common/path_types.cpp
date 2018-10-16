#include "hybrid_planning_common/path_types.h"

hybrid_planning_common::AlignedVector<Eigen::Isometry3d> hybrid_planning_common::flatten(const Path& path)
{
  hybrid_planning_common::AlignedVector<Eigen::Isometry3d> result;
  for (const auto& pass : path)
    result.insert(end(result), begin(pass), end(pass));
  return result;
}
