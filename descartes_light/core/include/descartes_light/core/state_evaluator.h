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
#ifndef DESCARTES_LIGHT_CORE_STATE_EVALUATOR_H
#define DESCARTES_LIGHT_CORE_STATE_EVALUATOR_H

#include <descartes_light/types.h>
#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
#include <Eigen/Geometry>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
template <typename FloatType>
class StateEvaluator
{
public:
  using Ptr = std::shared_ptr<StateEvaluator<FloatType>>;
  using ConstPtr = std::shared_ptr<const StateEvaluator<FloatType>>;

  StateEvaluator() = default;
  virtual ~StateEvaluator() = default;
  StateEvaluator(const StateEvaluator&) = default;
  StateEvaluator& operator=(const StateEvaluator&) = default;
  StateEvaluator(StateEvaluator&&) noexcept = default;
  StateEvaluator& operator=(StateEvaluator&&) noexcept = default;

  virtual std::pair<bool, FloatType> evaluate(const State<FloatType>& solution) const = 0;
};

using StateEvaluatorF = StateEvaluator<float>;
using StateEvaluatorD = StateEvaluator<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_CORE_STATE_COST_EVALUATOR_H
