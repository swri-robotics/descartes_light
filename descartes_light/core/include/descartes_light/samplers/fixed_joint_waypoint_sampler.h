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
#ifndef DESCARTES_SAMPLERS_SAMPLERS_FIXED_JOINT_WAYPOINT_SAMPLER_H
#define DESCARTES_SAMPLERS_SAMPLERS_FIXED_JOINT_WAYPOINT_SAMPLER_H

#include <descartes_light/core/waypoint_sampler.h>

namespace descartes_light
{
template <typename FloatType>
class FixedJointWaypointSampler : public WaypointSampler<FloatType>
{
public:
  // NOLINTNEXTLINE(modernize-pass-by-value)
  FixedJointWaypointSampler(typename State<FloatType>::ConstPtr fixed_joint_position);

  std::vector<StateSample<FloatType>> sample() const override;

private:
  typename State<FloatType>::ConstPtr fixed_joint_position_;
};

using FixedJointWaypointSamplerF = FixedJointWaypointSampler<float>;
using FixedJointWaypointSamplerD = FixedJointWaypointSampler<double>;

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_SAMPLERS_FIXED_JOINT_WAYPOINT_SAMPLER_H
