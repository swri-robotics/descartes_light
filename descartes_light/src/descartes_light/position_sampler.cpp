#include "descartes_light/position_sampler.h"
#include <iostream>

const static std::size_t opw_dof = 6;

descartes_light::CartesianPointSampler::CartesianPointSampler(const Eigen::Isometry3d& tool_pose,
                                                              const KinematicsInterface& robot_kin,
                                                              const CollisionInterfacePtr collision)
  : tool_pose_(tool_pose)
  , kin_(robot_kin)
  , collision_(std::move(collision))
{
}

bool descartes_light::CartesianPointSampler::sample(std::vector<double>& solution_set)
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

bool descartes_light::CartesianPointSampler::isCollisionFree(const double* vertex)
{
  return collision_->validate(vertex, opw_dof);
}

descartes_light::AxialSymmetricSampler::AxialSymmetricSampler(const Eigen::Isometry3d& tool_pose,
                                                              const descartes_light::KinematicsInterface& robot_kin,
                                                              const double radial_sample_resolution)
  : tool_pose_(tool_pose)
  , kin_(robot_kin)
  , radial_sample_res_(radial_sample_resolution)
{}

bool descartes_light::AxialSymmetricSampler::sample(std::vector<double>& solution_set)
{
  std::vector<double> buffer;

  const auto nSamplesInBuffer = [] (const std::vector<double>& v) -> std::size_t {
    return v.size() / opw_dof;
  };

  double angle = -M_PI;

  while (angle <= M_PI) // loop over each waypoint
  {
    Eigen::Isometry3d p = tool_pose_ * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    kin_.ik(p, buffer);

    const auto n_sols = nSamplesInBuffer(buffer);
    for (std::size_t i = 0; i < n_sols; ++i)
    {
      const auto* sol_data = buffer.data() + i * opw_dof;
      solution_set.insert(end(solution_set), sol_data, sol_data + opw_dof);
    }
    buffer.clear();

    angle += radial_sample_res_;
  } // redundancy resolution loop

  return !solution_set.empty();
}
