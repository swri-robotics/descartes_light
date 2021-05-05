/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2021, Southwest Research Institute
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
#pragma once

#include <descartes_light/state_evaluators/euclidean_distance_state_evaluator.h>

namespace descartes_light
{
template <typename FloatType>
EuclideanDistanceStateEvaluator<FloatType>::EuclideanDistanceStateEvaluator(
    const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& reference,
    const Eigen::Array<FloatType, Eigen::Dynamic, 1>& scale)
  : reference_(reference), scale_(scale)
{
  if (reference.size() != scale.size())
    throw std::runtime_error("Reference state and scale must be the same size");
}

template <typename FloatType>
EuclideanDistanceStateEvaluator<FloatType>::EuclideanDistanceStateEvaluator(
    const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& reference)
  : EuclideanDistanceStateEvaluator(reference, Eigen::Array<FloatType, Eigen::Dynamic, 1>::Ones(reference.size()))
{
}

template <typename FloatType>
std::pair<bool, FloatType>
EuclideanDistanceStateEvaluator<FloatType>::evaluate(const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& state) const
{
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> diff = (reference_ - state).array() * scale_;
  return std::make_pair(true, diff.squaredNorm());
}

}  // namespace descartes_light
