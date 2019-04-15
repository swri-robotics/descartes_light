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
#ifndef DESCARTES_LIGHT_EXTERNAL_AXIS_POSITION_SAMPLER_H
#define DESCARTES_LIGHT_EXTERNAL_AXIS_POSITION_SAMPLER_H

#include "descartes_light/core/collision_interface.h"
#include "descartes_light/core/kinematics_interface.h"
#include "descartes_light/core/position_sampler.h"

namespace descartes_light
{

template<typename FloatType>
class ExternalAxisSampler : public PositionSampler<FloatType>
{
public:
  ExternalAxisSampler(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool_in_positioner,
                      const typename KinematicsInterface<FloatType>::Ptr robot_kin,
                      const typename CollisionInterface<FloatType>::Ptr collision);

  bool sample(std::vector<FloatType>& solution_set) override;

private:
  bool isCollisionFree(const FloatType* vertex);

  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool_pose_;
  typename KinematicsInterface<FloatType>::Ptr kin_;
  typename CollisionInterface<FloatType>::Ptr collision_;
};

template<typename FloatType>
class SpoolSampler : public descartes_light::PositionSampler<FloatType>
{
public:
  SpoolSampler(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool_in_positioner,
               const typename KinematicsInterface<FloatType>::Ptr robot_kin,
               const typename CollisionInterface<FloatType>::Ptr collision);

  bool sample(std::vector<FloatType>& solution_set) override;

private:
  bool isCollisionFree(const FloatType* vertex);

  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool_pose_;
  typename KinematicsInterface<FloatType>::Ptr kin_;
  typename CollisionInterface<FloatType>::Ptr collision_;
};

} // namespace descartes_light

#endif
