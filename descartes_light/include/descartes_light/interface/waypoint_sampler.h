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

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
#include <Eigen/Geometry>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
/**
 * @brief For a given waypoint this should return a vector of solutions
 * @details Example for joint space planning
 */
template <typename FloatType>
class WaypointSampler
{
public:
  using Ptr = std::shared_ptr<WaypointSampler<FloatType>>;
  using ConstPtr = std::shared_ptr<const WaypointSampler<FloatType>>;

  virtual ~WaypointSampler() = default;

  virtual std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> sample() const = 0;
};

using WaypointSamplerF = WaypointSampler<float>;
using WaypointSamplerD = WaypointSampler<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_CORE_WAYPOINT_SAMPLER_H
