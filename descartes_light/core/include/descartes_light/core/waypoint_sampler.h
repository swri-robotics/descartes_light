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
#ifndef DESCARTES_LIGHT_CORE_WAYPOINT_SAMPLER_H
#define DESCARTES_LIGHT_CORE_WAYPOINT_SAMPLER_H

#include <descartes_light/types.h>
#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
#include <Eigen/Geometry>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
/**
 * @brief Produces a set of states for which a waypoint is valid
 */
template <typename FloatType>
class WaypointSampler
{
public:
  using Ptr = std::shared_ptr<WaypointSampler<FloatType>>;
  using ConstPtr = std::shared_ptr<const WaypointSampler<FloatType>>;

  WaypointSampler() = default;
  virtual ~WaypointSampler() = default;
  WaypointSampler(const WaypointSampler&) = default;
  WaypointSampler& operator=(const WaypointSampler&) = default;
  WaypointSampler(WaypointSampler&&) noexcept = default;
  WaypointSampler& operator=(WaypointSampler&&) noexcept = default;

  /**
   * @brief Samples a waypoint to produce a list of states for which the waypoint is valid along with an initial cost
   * @details This function allows an initial cost to be assigned to each state sample. This cost is functionally the
   * same as the cost provided by the StateEvaluator, but in some cases it can be more efficient to calculate certain
   * costs during the sample step.
   *
   * Consider calculating a cost based on the distance from nearest collision for a robot with redundant joint
   * states. Collision checking is computationally expensive and, as such, should only be evaluated for the fewest
   * number of states possible. Evaluating distance from nearest collision as a StateEvalutor would result in a
   * significant amount of repeat work since all redundant states would produce the same cost. Therefore it is
   * preferable to generate this cost once for the nominal state and apply it to all redundant states produced in the
   * sampling step
   */
  virtual typename std::vector<StateSample<FloatType>> sample() const = 0;
};

using WaypointSamplerF = WaypointSampler<float>;
using WaypointSamplerD = WaypointSampler<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_CORE_WAYPOINT_SAMPLER_H
