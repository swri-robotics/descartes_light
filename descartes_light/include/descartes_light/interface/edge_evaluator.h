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
#ifndef DESCARTES_LIGHT_CORE_EDGE_EVALUATOR_H
#define DESCARTES_LIGHT_CORE_EDGE_EVALUATOR_H

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
DESCARTES_IGNORE_WARNINGS_POP

#include "descartes_light/ladder_graph.h"

namespace descartes_light
{
template <typename FloatType>
class EdgeEvaluator
{
public:
  using Ptr = typename std::shared_ptr<EdgeEvaluator<FloatType>>;
  using ConstPtr = typename std::shared_ptr<const EdgeEvaluator<FloatType>>;

  virtual ~EdgeEvaluator() = default;

  /**
   * @brief Determines whether the edge between two vertices is valid and, if so, its cost.
   * @param start The start state of the edge
   * @param end The end state of the edge
   * @return A pair <True/False, Cost>, True if edge is valid, false otherwise. Cost to move from the first vertex to
   * the next
   */
  virtual std::pair<bool, FloatType> evaluate(const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& start,
                                              const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& end) const = 0;
};

using EdgeEvaluatorF = EdgeEvaluator<float>;
using EdgeEvaluatorD = EdgeEvaluator<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_CORE_EDGE_EVALUATOR_H
