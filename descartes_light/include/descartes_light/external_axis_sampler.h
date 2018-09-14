#ifndef EXTERNAL_AXIS_POSITION_SAMPLER_H
#define EXTERNAL_AXIS_POSITION_SAMPLER_H

#include "descartes_light/position_sampler.h"

namespace descartes_light
{

class ExternalAxisSampler : public PositionSampler
{
public:
  ExternalAxisSampler(const Eigen::Isometry3d& tool_pose_on_axis,
                      const KinematicsInterface& robot_kin,
                      const CollisionInterfacePtr collision);

  bool sample(std::vector<double>& solution_set) override;

private:
  bool isCollisionFree(const double* vertex);

  Eigen::Isometry3d tool_pose_;
  KinematicsInterface kin_;
  CollisionInterfacePtr collision_;
};


}

#endif
