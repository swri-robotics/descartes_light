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
#ifndef DESCARTES_LIGHT_SOLVERS_BGL_IMPL_DFS_SOLVER_HPP
#define DESCARTES_LIGHT_SOLVERS_BGL_IMPL_DFS_SOLVER_HPP

#include <descartes_light/descartes_macros.h>

DESCARTES_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <omp.h>
#include <boost/graph/depth_first_search.hpp>
#include <descartes_light/solvers/bgl/bgl_dfs_solver.h>
#include <descartes_light/solvers/bgl/impl/event_visitors.hpp>
#include <descartes_light/types.h>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
template <typename FloatType>
SearchResult<FloatType> DepthFirstSVDESolver<FloatType>::search()
{
  SearchResult<FloatType> result;

  // Internal properties
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  const auto& source_ = BGLSolverBase<FloatType>::source_;
  auto& predecessors_ = BGLSolverBase<FloatType>::predecessors_;
  auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;
  auto& edge_eval_ = BGLSolverBaseSVDE<FloatType>::edge_eval_;

  result.cost = std::numeric_limits<FloatType>::max();
  result.trajectory = {};

  predecessors_.resize(boost::num_vertices(graph_), std::numeric_limits<std::size_t>::max());
  std::vector<VertexDesc<FloatType>> dist_vec(boost::num_vertices(graph_), std::numeric_limits<std::size_t>::max());
  auto color_prop_map = boost::get(&Vertex<FloatType>::color, graph_);

  const long last_rung_idx = static_cast<long>(ladder_rungs_.size() - 1);
  auto visitor = boost::make_dfs_visitor(std::make_pair(
      early_terminator_discover<FloatType>(last_rung_idx),
      std::make_pair(add_all_edges_dynamically<FloatType>(edge_eval_, ladder_rungs_),
                     std::make_pair(boost::record_predecessors(predecessors_.data(), boost::on_tree_edge()),
                                    cost_recorder<FloatType>()))));

  try
  {
    graph_[source_].distance = 0.0;

    boost::depth_first_search(graph_, visitor, color_prop_map, source_);
  }
  catch (const VertexDesc<FloatType>& target)
  {
    // the source point is always the last vertex inthe vector
    predecessors_[predecessors_.size() - 1] = predecessors_.size() - 1;
    const auto valid_path = BGLSolverBase<FloatType>::reconstructPath(source_, target);
    result.trajectory = BGLSolverBase<FloatType>::toStates(valid_path);

    // remove empty start state
    result.trajectory.erase(result.trajectory.begin());
    result.cost = graph_[target].distance;

    return result;
  }

  throw std::runtime_error("DFS search failed to encounter a vertex in the last rung of the plan graph");
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_IMPL_DFS_SOLVER_HPP
