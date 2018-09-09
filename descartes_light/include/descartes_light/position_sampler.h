#ifndef DESCARTES_LIGHT_POSITION_SAMPLER_H
#define DESCARTES_LIGHT_POSITION_SAMPLER_H

#include "descartes_light/kinematic_interface.h"
#include <memory>

namespace descartes_light
{

// Section 1: Sampler
class PositionSampler
{
public:
  virtual ~PositionSampler() {}
  virtual bool sample(std::vector<double>& solution_set) = 0;
};

class CartesianPointSampler : public PositionSampler
{
public:
  CartesianPointSampler(const Eigen::Isometry3d& tool_pose);

  bool sample(std::vector<double>& solution_set) override;

private:
  Eigen::Isometry3d tool_pose_;
  KinematicsInterface kin_;
};

using PositionSamplerPtr = std::shared_ptr<PositionSampler>;


}

#endif // DESCARTES_LIGHT_POSITION_SAMPLER_H
