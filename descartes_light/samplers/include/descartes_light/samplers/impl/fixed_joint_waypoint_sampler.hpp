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
#ifndef DESCARTES_SAMPLERS_SAMPLERS_IMPL_FIXED_JOINT_WAYPOINT_SAMPLER_HPP
#define DESCARTES_SAMPLERS_SAMPLERS_IMPL_FIXED_JOINT_WAYPOINT_SAMPLER_HPP

#include <descartes_light/samplers/fixed_joint_waypoint_sampler.h>

namespace descartes_light
{
template <typename FloatType>
FixedJointWaypointSampler<FloatType>::FixedJointWaypointSampler(
    typename State<FloatType>::ConstPtr fixed_joint_position)
  : fixed_joint_position_(std::move(fixed_joint_position))
{
}

template <typename FloatType>
std::vector<StateSample<FloatType>> FixedJointWaypointSampler<FloatType>::sample() const
{
  return { StateSample<FloatType>{ fixed_joint_position_, static_cast<FloatType>(0.0) } };
}

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_SAMPLERS_IMPL_FIXED_JOINT_WAYPOINT_SAMPLER_HPP
