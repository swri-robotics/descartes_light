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
#ifndef DESCARTES_SAMPLERS_SAMPLERS_FIXED_JOINT_POSE_SAMPLER_H
#define DESCARTES_SAMPLERS_SAMPLERS_FIXED_JOINT_POSE_SAMPLER_H

#include <descartes_light/interface/position_sampler.h>

namespace descartes_light
{
template <typename FloatType>
class FixedJointPoseSampler : public PositionSampler<FloatType>
{
public:
  FixedJointPoseSampler(const std::vector<FloatType>& fixed_joint_position);

  bool sample(std::vector<FloatType>& solution_set) override;

private:
  std::vector<FloatType> fixed_joint_position_;
};

using FixedJointPoseSamplerF = FixedJointPoseSampler<float>;
using FixedJointPoseSamplerD = FixedJointPoseSampler<double>;

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_SAMPLERS_FIXED_JOINT_POSE_SAMPLER_H
