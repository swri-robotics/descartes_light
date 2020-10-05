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
  EdgeEvaluator(std::size_t dof) : dof_(dof) {}
  virtual ~EdgeEvaluator() {}

  bool evaluate(const Rung_<FloatType>& from,
                const Rung_<FloatType>& to,
                std::vector<typename LadderGraph<FloatType>::EdgeList>& edge_lists)
  {
    const auto n_start = from.data.size() / dof_;
    const auto n_end = to.data.size() / dof_;

    edge_lists.resize(n_start);
    for (std::size_t i = 0; i < n_start; ++i)
    {
      const auto* start_vertex = from.data.data() + dof_ * i;
      for (std::size_t j = 0; j < n_end; ++j)
      {
        const FloatType* end_vertex = to.data.data() + dof_ * j;

        // Consider the edge:
        std::pair<bool, FloatType> results = considerEdge(start_vertex, end_vertex);
        if (results.first)
          edge_lists[i].emplace_back(results.second, j);
      }
    }

    for (const auto& edge_list : edge_lists)
      if (!edge_list.empty())
        return true;

    return false;
  }

  /**
   * @brief Determines whether the edge between two vertices is valid and, if so, its cost.
   * @param start The start state of the edge
   * @param end The end state of the edge
   * @return A pair <True/False, Cost>, True if edge is valid, false otherwise. Cost to move from the first vertex to
   * the next
   */
  virtual std::pair<bool, FloatType> considerEdge(const FloatType* start, const FloatType* end) = 0;

protected:
  std::size_t dof_;
};

using EdgeEvaluatorF = EdgeEvaluator<float>;
using EdgeEvaluatorD = EdgeEvaluator<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_CORE_EDGE_EVALUATOR_H
