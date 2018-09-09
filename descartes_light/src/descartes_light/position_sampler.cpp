#include "descartes_light/position_sampler.h"
descartes_light::CartesianPointSampler::CartesianPointSampler(const Eigen::Isometry3d& tool_pose)
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

  }

  return false;
}
