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
#ifndef DESCARTES_LIGHT_RAIL_POSITION_SAMPLER_H
#define DESCARTES_LIGHT_RAIL_POSITION_SAMPLER_H

#include "descartes_light/core/kinematics_interface.h"
#include "descartes_light/core/collision_interface.h"
#include "descartes_light/core/position_sampler.h"
#include <memory>

namespace descartes_light
{

class RailedCartesianPointSampler : public PositionSampler
{
public:
  RailedCartesianPointSampler(const Eigen::Isometry3d& tool_pose,
                              const KinematicsInterfacePtr robot_kin,
                              const CollisionInterfacePtr collision);

  /**
   * @brief sample
   * @param solution_set Returns auxiliary axes followed by robot positions
   * @return True if valid solutions were found, otherwise false.
   */
  bool sample(std::vector<double>& solution_set) override;

private:
  bool isCollisionFree(const double* vertex);

  Eigen::Isometry3d tool_pose_;
  KinematicsInterfacePtr kin_;
  CollisionInterfacePtr collision_;
};

}

#endif
