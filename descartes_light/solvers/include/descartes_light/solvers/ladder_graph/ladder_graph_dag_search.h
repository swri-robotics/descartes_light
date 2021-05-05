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

#ifndef DESCARTES_LIGHT_LADDER_GRAPH_DAG_SEARCH_H
#define DESCARTES_LIGHT_LADDER_GRAPH_DAG_SEARCH_H

#include <descartes_light/solvers/ladder_graph/ladder_graph.h>

namespace descartes_light
{
template <typename FloatType>
class DAGSearch
{
public:
  using predecessor_t = unsigned;
  using size_type = std::size_t;

  explicit DAGSearch(const LadderGraph<FloatType>& graph);

  FloatType run();

  std::vector<predecessor_t> shortestPath() const;

private:
  const LadderGraph<FloatType>& graph_;

  struct SolutionRung
  {
    std::vector<FloatType> distance;
    std::vector<predecessor_t> predecessor;
  };

  inline FloatType& distance(size_type rung, size_type index) noexcept { return solution_[rung].distance[index]; }

  inline predecessor_t& predecessor(size_type rung, size_type index) noexcept
  {
    return solution_[rung].predecessor[index];
  }

  inline const predecessor_t& predecessor(size_type rung, size_type index) const noexcept
  {
    return solution_[rung].predecessor[index];
  }

  std::vector<SolutionRung> solution_;
};

using DAGSearchF = DAGSearch<float>;
using DAGSearchD = DAGSearch<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_LADDER_GRAPH_DAG_SEARCH_H
