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
#ifndef DESCARTES_SAMPLERS_EVALUATORS_IMPL_TIMING_EDGE_EVALUATOR_HPP
#define DESCARTES_SAMPLERS_EVALUATORS_IMPL_TIMING_EDGE_EVALUATOR_HPP

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <cmath>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_samplers/evaluators/timing_edge_evaluator.h>

namespace descartes_light
{
template <typename FloatType>
TimingEdgeEvaluator<FloatType>::TimingEdgeEvaluator(const std::vector<FloatType>& velocity_limits,
                                                    FloatType dt,
                                                    FloatType safety_factor)
  : EdgeEvaluator<FloatType>(velocity_limits.size())
  , velocity_limits_(Eigen::Map<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>(
        velocity_limits.data(),
        static_cast<Eigen::Index>(velocity_limits.size())))
  , dt_(dt)
  , safety_factor_(safety_factor)
{
}

template <typename FloatType>
std::pair<bool, FloatType> TimingEdgeEvaluator<FloatType>::considerEdge(const FloatType* start, const FloatType* end)
{
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> delta =
      Eigen::Map<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>(end, static_cast<Eigen::Index>(this->dof_)) -
      Eigen::Map<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>(start, static_cast<Eigen::Index>(this->dof_));
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> joint_times = delta.cwiseQuotient(velocity_limits_).cwiseAbs();
  FloatType cost = joint_times.maxCoeff();

  return std::make_pair((cost < (safety_factor_ * dt_)), cost);
}

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_EVALUATORS_IMPL_TIMING_EDGE_EVALUATOR_HPP
