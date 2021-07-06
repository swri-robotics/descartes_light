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
#ifndef DESCARTES_SAMPLERS_EVALUATORS_COMPOUND_EDGE_EVALUATOR_H
#define DESCARTES_SAMPLERS_EVALUATORS_COMPOUND_EDGE_EVALUATOR_H

#include <descartes_light/core/edge_evaluator.h>

namespace descartes_light
{
template <typename FloatType>
class CompoundEdgeEvaluator : public EdgeEvaluator<FloatType>
{
public:
  CompoundEdgeEvaluator() = default;

  std::pair<bool, FloatType> evaluate(const State<FloatType>& start, const State<FloatType>& end) const override;

  std::vector<typename EdgeEvaluator<FloatType>::ConstPtr> evaluators;
};

using CompoundEdgeEvaluatorF = CompoundEdgeEvaluator<float>;
using CompoundEdgeEvaluatorD = CompoundEdgeEvaluator<double>;

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_EVALUATORS_COMPOUND_EDGE_EVALUATOR_H
