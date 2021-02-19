/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2020, Southwest Research Institute
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
#ifndef DESCARTES_SAMPLERS_EVALUATORS_COMPOUND_EDGE_EVALUATOR_HPP
#define DESCARTES_SAMPLERS_EVALUATORS_COMPOUND_EDGE_EVALUATOR_HPP

#include <descartes_samplers/evaluators/compound_edge_evaluator.h>

namespace descartes_light
{
template <typename FloatType>
std::pair<bool, FloatType>
CompoundEdgeEvaluator<FloatType>::evaluate(const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& start,
                                           const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& end) const
{
  FloatType cost = 0.0;
  for (auto& evaluator : evaluators)
  {
    auto results = evaluator->evaluate(start, end);
    if (!results.first)
      return std::make_pair(false, cost);

    cost += results.second;
  }

  return std::make_pair(true, cost);
}

}  // namespace descartes_light
#endif  // DESCARTES_SAMPLERS_EVALUATORS_COMPOUND_EDGE_EVALUATOR_HPP
