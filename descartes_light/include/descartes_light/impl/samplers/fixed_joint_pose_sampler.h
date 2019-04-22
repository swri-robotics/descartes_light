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
#ifndef DESCARTES_LIGHT_FIXED_JOINT_POSE_SAMPLER_H
#define DESCARTES_LIGHT_FIXED_JOINT_POSE_SAMPLER_H

#include "descartes_light/core/position_sampler.h"

namespace descartes_light
{

class FixedJointPoseSampler : public PositionSampler
{
public:

  FixedJointPoseSampler(const std::vector<double>& fixed_joint_position) : fixed_joint_position_(fixed_joint_position) {}

  bool sample(std::vector<double>& solution_set) override
  {
    solution_set.insert(solution_set.end(), fixed_joint_position_.begin(), fixed_joint_position_.end());
    return true;
  }

private:
  std::vector<double> fixed_joint_position_;

};

}
#endif // DESCARTES_LIGHT_FIXED_JOINT_POSE_SAMPLER_H
