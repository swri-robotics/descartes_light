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
#ifndef DESCARTES_SAMPLERS_EVALUATORS_IMPL_DISTANCE_EDGE_EVALUATOR_HPP
#define DESCARTES_SAMPLERS_EVALUATORS_IMPL_DISTANCE_EDGE_EVALUATOR_HPP

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <cmath>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_samplers/evaluators/distance_edge_evaluator.h>

namespace descartes_light
{
template <typename FloatType>
DistanceEdgeEvaluator<FloatType>::DistanceEdgeEvaluator(const std::vector<FloatType>& velocity_limits, FloatType dt)
  : EdgeEvaluator<FloatType>(velocity_limits.size())
{
  joint_distance_threshold_.resize(velocity_limits.size());
  std::vector<FloatType> joint_distance_threshold(velocity_limits.size());
  std::transform(velocity_limits.begin(),
                 velocity_limits.end(),
                 joint_distance_threshold_.begin(),
                 [dt](const FloatType& vel_limit) -> FloatType {
                   if (dt != static_cast<FloatType>(0.0))
                   {
                     const static FloatType safety_factor = static_cast<FloatType>(0.9);
                     return dt * vel_limit * safety_factor;
                   }
                   else
                   {
                     return std::numeric_limits<FloatType>::max();
                   }
                 });
}

template <typename FloatType>
std::pair<bool, FloatType> DistanceEdgeEvaluator<FloatType>::considerEdge(const FloatType* start, const FloatType* end)
{
  FloatType cost = static_cast<FloatType>(0.0);
  for (std::size_t i = 0; i < joint_distance_threshold_.size(); ++i)
  {
    FloatType step = end[i] - start[i];
    if (std::abs(step) > joint_distance_threshold_[i])
      return std::make_pair(false, cost);

    cost += std::pow(step, FloatType(2));
  }

  return std::make_pair(true, cost);
}

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_EVALUATORS_IMPL_DISTANCE_EDGE_EVALUATOR_HPP
