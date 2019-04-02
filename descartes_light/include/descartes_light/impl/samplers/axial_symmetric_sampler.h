/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef DESCARTES_LIGHT_AXIAL_SYMMETRIC_SAMPLER_H
#define DESCARTES_LIGHT_AXIAL_SYMMETRIC_SAMPLER_H

#include "descartes_light/core/kinematics_interface.h"
#include "descartes_light/core/collision_interface.h"
#include "descartes_light/core/position_sampler.h"

namespace descartes_light
{

class AxialSymmetricSampler : public PositionSampler
{
public:
  AxialSymmetricSampler(const Eigen::Isometry3d& tool_pose,
                        const KinematicsInterfacePtr robot_kin,
                        const double radial_sample_resolution,
                        const CollisionInterfacePtr collision);

  bool sample(std::vector<double>& solution_set) override;

private:
  bool isCollisionFree(const double* vertex);

  Eigen::Isometry3d tool_pose_;
  KinematicsInterfacePtr kin_;
  CollisionInterfacePtr collision_;
  double radial_sample_res_;
};

}

#endif // DESCARTES_LIGHT_AXIAL_SYMMETRIC_SAMPLER_H
