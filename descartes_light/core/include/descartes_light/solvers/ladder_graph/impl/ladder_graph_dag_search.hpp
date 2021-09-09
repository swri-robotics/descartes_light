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
#ifndef DESCARTES_LIGHT_IMPL_LADDER_GRAPH_DAG_SEARCH_HPP
#define DESCARTES_LIGHT_IMPL_LADDER_GRAPH_DAG_SEARCH_HPP

#include <descartes_light/solvers/ladder_graph/ladder_graph_dag_search.h>

namespace descartes_light
{
template <typename FloatType>
DAGSearch<FloatType>::DAGSearch(const LadderGraph<FloatType>& graph) : graph_(graph)
{
  // On creating an object, let's allocate everything we need
  solution_.resize(graph.size());

  for (size_t i = 0; i < graph.size(); ++i)
  {
    const auto n_vertices = graph.rungSize(i);
    if (n_vertices == 0)
      throw std::runtime_error("Ladder graph rung " + std::to_string(i) + " has no vertices");

    solution_[i].distance.resize(n_vertices);
    solution_[i].predecessor.resize(n_vertices);
  }
}

template <typename FloatType>
FloatType DAGSearch<FloatType>::run()
{
  // Cost to the first rung should be set to zero
  std::fill(solution_.front().distance.begin(), solution_.front().distance.end(), 0.0);

  // Other rows initialize to numeric limits max
  for (size_type i = 1; i < solution_.size(); ++i)
  {
    std::fill(solution_[i].distance.begin(), solution_[i].distance.end(), std::numeric_limits<FloatType>::max());
  }

  // Now we iterate over the graph in 'topological' order
  for (size_type r = 0; r < solution_.size() - 1; ++r)
  {
    const auto next_r = r + 1;
    const auto& rung = graph_.getRung(r);
    const auto& next_rung = graph_.getRung(next_r);

    // For each node in the out edge list
    for (size_t n = 0; n < rung.nodes.size(); ++n)
    {
      const auto& node = rung.nodes[n];

      // If first rung then the cost is the node cost else lookup cost
      const FloatType u_cost = (r == 0) ? node.sample.cost : distance(r, n);

      // for each out edge
      for (const auto& edge : node.edges)
      {
        // new cost = edge cost + node cost
        auto dv = u_cost + edge.cost + next_rung.nodes[edge.idx].sample.cost;
        if (dv < distance(next_r, edge.idx))
        {
          distance(next_r, edge.idx) = dv;
          // the predecessor's rung is implied to be the current rung
          predecessor(next_r, edge.idx) = static_cast<unsigned>(n);
        }
      }
    }  // node for loop
  }    // rung for loop

  auto it = std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
  if (it != solution_.back().distance.end())
    return *it;

  throw std::runtime_error("Failed to get minimum cost from the last rung of the ladder graph");
}

template <typename FloatType>
std::vector<typename DAGSearch<FloatType>::predecessor_t> DAGSearch<FloatType>::shortestPath() const
{
  auto min_it = std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
  auto min_idx = std::distance(solution_.back().distance.begin(), min_it);
  assert(min_idx >= 0);

  std::vector<predecessor_t> path(solution_.size());

  size_type current_rung = path.size() - 1;
  auto current_index = static_cast<size_type>(min_idx);

  for (unsigned i = 0; i < path.size(); ++i)
  {
    auto count = path.size() - 1 - i;
    assert(current_rung == count);
    path[count] = static_cast<unsigned>(current_index);
    current_index = predecessor(current_rung, current_index);
    current_rung -= 1;
  }

  return path;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_IMPL_LADDER_GRAPH_DAG_SEARCH_HPP
