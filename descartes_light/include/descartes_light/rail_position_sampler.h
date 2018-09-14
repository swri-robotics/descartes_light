#ifndef RAIL_POSITION_SAMPLER_H
#define RAIL_POSITION_SAMPLER_H

#include "descartes_light/kinematic_interface.h"
#include "descartes_light/collision_checker.h"
#include "descartes_light/position_sampler.h"
#include <memory>

namespace descartes_light
{

class RailedCartesianPointSampler : public PositionSampler
{
public:
  RailedCartesianPointSampler(const Eigen::Isometry3d& tool_pose,
                              const RailedKinematicsInterface& robot_kin,
                              const CollisionInterfacePtr collision);

  bool sample(std::vector<double>& solution_set) override;

private:
  bool isCollisionFree(const double* vertex);

  Eigen::Isometry3d tool_pose_;
  RailedKinematicsInterface kin_;
  CollisionInterfacePtr collision_;
};

class RailedAxialSymmetricSampler : public PositionSampler
{
public:
  RailedAxialSymmetricSampler(const Eigen::Isometry3d& tool_pose,
                              const RailedKinematicsInterface& robot_kin,
                              const double radial_sample_resolution,
                              const CollisionInterfacePtr collision);

  bool sample(std::vector<double>& solution_set) override;

private:
  bool isCollisionFree(const double* vertex);

  Eigen::Isometry3d tool_pose_;
  RailedKinematicsInterface kin_;
  CollisionInterfacePtr collision_;
  double radial_sample_res_;
};

}

#endif // RAIL_POSITION_SAMPLER_H
