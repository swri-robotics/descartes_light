#include "descartes_light/external_axis_sampler.h"
#include <iostream>

const static std::size_t opw_dof = 7;

descartes_light::ExternalAxisSampler::ExternalAxisSampler(const Eigen::Isometry3d& tool_pose,
                                                          const KinematicsInterface& robot_kin,
                                                          const CollisionInterfacePtr collision)
  : tool_pose_(tool_pose)
  , kin_(robot_kin)
  , collision_(std::move(collision))
{
}

bool descartes_light::ExternalAxisSampler::sample(std::vector<double>& solution_set)
{
  std::vector<double> buffer;
  kin_.ik(tool_pose_, buffer);

  const auto nSamplesInBuffer = [] (const std::vector<double>& v) -> std::size_t {
    return v.size() / opw_dof;
  };

  const auto n_sols = nSamplesInBuffer(buffer);

  for (std::size_t i = 0; i < n_sols; ++i)
  {
    const auto* sol_data = buffer.data() + i * opw_dof;
    if (isCollisionFree(sol_data))
      solution_set.insert(end(solution_set), sol_data, sol_data + opw_dof);
  }

  return !solution_set.empty();
}

bool descartes_light::ExternalAxisSampler::isCollisionFree(const double* vertex)
{
  return collision_->validate(vertex, opw_dof);
}
