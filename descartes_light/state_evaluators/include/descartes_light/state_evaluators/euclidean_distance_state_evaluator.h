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

#include <descartes_light/core/state_evaluator.h>

namespace descartes_light
{
/**
 * @brief Computes the cost of a state as the squared Euclidean distance between it and a reference state
 * This distance can also be multiplied by an input scale factor in the case of mixed units within the elements of the
 * state (such as linear and revolute joints of a robot)
 */
template <typename FloatType>
class EuclideanDistanceStateEvaluator : public StateEvaluator<FloatType>
{
public:
  EuclideanDistanceStateEvaluator(const Eigen::Ref<const State<FloatType>>& reference,
                                  const Eigen::Ref<const Eigen::Array<FloatType, Eigen::Dynamic, 1>>& scale);
  EuclideanDistanceStateEvaluator(const Eigen::Ref<const State<FloatType>>& reference);

  std::pair<bool, FloatType> evaluate(const Eigen::Ref<const State<FloatType>>& state) const override;

private:
  const State<FloatType> reference_;
  const Eigen::Array<FloatType, Eigen::Dynamic, 1> scale_;
};

using EuclideanDistanceStateEvaluatorD = EuclideanDistanceStateEvaluator<double>;
using EuclideanDistanceStateEvaluatorF = EuclideanDistanceStateEvaluator<float>;

}  // namespace descartes_light
