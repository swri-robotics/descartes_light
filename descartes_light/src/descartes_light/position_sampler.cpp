#include "descartes_light/position_sampler.h"
#include <iostream>

descartes_light::CartesianPointSampler::CartesianPointSampler(const Eigen::Isometry3d& tool_pose, const KinematicsInterface& robot_kin)
  : tool_pose_(tool_pose)
  , kin_(robot_kin)
{
}

bool descartes_light::CartesianPointSampler::sample(std::vector<double>& solution_set)
{
  std::vector<double> buffer;
  kin_.ik(tool_pose_, buffer);

  const auto nSamplesInBuffer = [] (const std::vector<double>& v) -> std::size_t {
    return v.size() / 6;
  };

  const auto n_sols = nSamplesInBuffer(buffer);

  for (std::size_t i = 0; i < n_sols; ++i)
  {
    // TODO: Collision?
    solution_set.insert(end(solution_set), begin(buffer) + i * 6, begin(buffer) + (i+1) * 6);
    buffer[i*6 + 5] += 2 * M_PI;
    solution_set.insert(end(solution_set), begin(buffer) + i * 6, begin(buffer) + (i+1) * 6);
    buffer[i*6 + 5] -= 4 * M_PI;
    solution_set.insert(end(solution_set), begin(buffer) + i * 6, begin(buffer) + (i+1) * 6);
    buffer[i*6 + 5] += 2 * M_PI;

//    solution_set.insert(end(solution_set), begin(buffer) + i * 6, begin(buffer) + (i+1) * 6);
    buffer[i*6 + 3] += 2 * M_PI;
    solution_set.insert(end(solution_set), begin(buffer) + i * 6, begin(buffer) + (i+1) * 6);
    buffer[i*6 + 3] -= 4 * M_PI;
    solution_set.insert(end(solution_set), begin(buffer) + i * 6, begin(buffer) + (i+1) * 6);

  }

  return !solution_set.empty();
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
//  std::cout << "**************************************\n\n";
  std::vector<double> buffer;

  const auto nSamplesInBuffer = [] (const std::vector<double>& v) -> std::size_t {
    return v.size() / 6;
  };

  double angle = -M_PI;

  while (angle <= M_PI)
  {
    Eigen::Isometry3d p = tool_pose_ * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
//    std::cout << p.matrix() << "\n\n";
    kin_.ik(p, buffer);
    const auto n_sols = nSamplesInBuffer(buffer);
    for (std::size_t i = 0; i < n_sols; ++i)
    {
      // TODO: Collision?
      solution_set.insert(end(solution_set), begin(buffer) + i * 6, begin(buffer) + (i+1) * 6);
      buffer[i*6 + 5] += 2 * M_PI;
      solution_set.insert(end(solution_set), begin(buffer) + i * 6, begin(buffer) + (i+1) * 6);
      buffer[i*6 + 5] -= 4 * M_PI;
      solution_set.insert(end(solution_set), begin(buffer) + i * 6, begin(buffer) + (i+1) * 6);
    }
    buffer.clear();

    angle += radial_sample_res_;
  }

  return !solution_set.empty();
}
