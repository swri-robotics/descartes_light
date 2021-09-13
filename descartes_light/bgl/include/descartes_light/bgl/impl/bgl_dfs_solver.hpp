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

#include <descartes_light/bgl/bgl_dfs_solver.h>
#include <descartes_light/bgl/impl/event_visitors.hpp>

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <boost/graph/depth_first_search.hpp>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
template <typename FloatType, typename Visitors>
static VertexDesc<FloatType> solveDFS(BGLGraph<FloatType>& graph,
                                      const VertexDesc<FloatType>& source,
                                      const Visitors& event_visitors,
                                      const std::vector<std::vector<VertexDesc<FloatType>>>& ladder_rungs)
{
  auto color_prop_map = boost::get(&Vertex<FloatType>::color, graph);
  auto visitor = boost::make_dfs_visitor(event_visitors);

  try
  {
    graph[source].distance = 0.0;
    boost::depth_first_search(graph, visitor, color_prop_map, source);

    // In the case that the visitor does not throw the target vertex descriptor, find the lowest cost vertex in last
    // rung of the ladder graph
    auto target = std::min_element(ladder_rungs.back().begin(),
                                   ladder_rungs.back().end(),
                                   [&](const VertexDesc<FloatType>& a, const VertexDesc<FloatType>& b) {
                                     return graph[a].distance < graph[b].distance;
                                   });

    // Check that the identified lowest cost vertex is valid and has a cost less than inf
    if (target != ladder_rungs.back().end() && graph[*target].distance < std::numeric_limits<FloatType>::max())
      throw *target;
  }
  catch (const VertexDesc<FloatType>& target)
  {
    return target;
  }

  throw std::runtime_error("DFS search failed to encounter a vertex in the last rung of the plan graph");
}

template <typename FloatType, typename Visitors>
BGLDepthFirstSVSESolver<FloatType, Visitors>::BGLDepthFirstSVSESolver(Visitors event_visitors, unsigned num_threads)
  : BGLSolverBaseSVSE<FloatType>(num_threads), event_visitors_(std::move(event_visitors))
{
}

template <typename FloatType, typename Visitors>
SearchResult<FloatType> BGLDepthFirstSVSESolver<FloatType, Visitors>::search()
{
  // Internal properties
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  const auto& source_ = BGLSolverBase<FloatType>::source_;
  auto& predecessors_ = BGLSolverBase<FloatType>::predecessors_;
  const auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;

  // Resize the container of predecessors to match the number of vertices
  // Assign the initial values as a vertex descriptor that is definitely not in the graph to avoid mistakes in the
  // future reconstructing the path
  predecessors_.resize(boost::num_vertices(graph_), std::numeric_limits<VertexDesc<FloatType>>::max());

  // Assign the predecessor of the source node to itself since the BGL DFS will not do it
  predecessors_.at(source_) = source_;

  // Create an event visitor that combines the internally specified visitors with ones that record predecessors and
  // vertex costs (i.e. distances)
  auto visitor = std::make_pair(boost::record_predecessors(predecessors_.data(), boost::on_tree_edge()),
                                std::make_pair(cost_recorder(), event_visitors_));

  const VertexDesc<FloatType> target = solveDFS(graph_, source_, visitor, ladder_rungs_);

  SearchResult<FloatType> result;

  // Reconstruct the path from the predecesor map; remove the artificial start state
  const auto vd_path = BGLSolverBase<FloatType>::reconstructPath(source_, target);
  result.trajectory = BGLSolverBase<FloatType>::toStates(vd_path);
  result.trajectory.erase(result.trajectory.begin());

  result.cost = graph_[target].distance;

  return result;
}

template <typename FloatType, typename Visitors>
BGLDepthFirstSVDESolver<FloatType, Visitors>::BGLDepthFirstSVDESolver(Visitors event_visitors, unsigned num_threads)
  : BGLSolverBaseSVDE<FloatType>(num_threads), event_visitors_(std::move(event_visitors))
{
}

template <typename FloatType, typename Visitors>
SearchResult<FloatType> BGLDepthFirstSVDESolver<FloatType, Visitors>::search()
{
  // Internal properties
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  const auto& source_ = BGLSolverBase<FloatType>::source_;
  auto& predecessors_ = BGLSolverBase<FloatType>::predecessors_;
  const auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;

  // Resize the container of predecessors to match the number of vertices
  // Assign the initial values as a vertex descriptor that is definitely not in the graph to avoid mistakes in the
  // future reconstructing the path
  predecessors_.resize(boost::num_vertices(graph_), std::numeric_limits<VertexDesc<FloatType>>::max());

  // Assign the predecessor of the source node to itself since the BGL DFS will not do it
  predecessors_.at(source_) = source_;

  // Make a visitor that combines the internally specified event visitors with ones that add all edges dynamically and
  // record predecessors and vertex costs (i.e. distances)
  const auto& edge_eval_ = BGLSolverBaseSVDE<FloatType>::edge_eval_;
  auto visitor = std::make_pair(
      boost::record_predecessors(predecessors_.data(), boost::on_tree_edge()),
      std::make_pair(
          cost_recorder(),
          std::make_pair(add_all_edges_dynamically<FloatType, boost::on_discover_vertex>(edge_eval_, ladder_rungs_),
                         event_visitors_)));

  const VertexDesc<FloatType> target = solveDFS(graph_, source_, visitor, ladder_rungs_);

  SearchResult<FloatType> result;

  // Reconstruct the path from the predecesor map; remove the artificial start state
  const auto vd_path = BGLSolverBase<FloatType>::reconstructPath(source_, target);
  result.trajectory = BGLSolverBase<FloatType>::toStates(vd_path);
  result.trajectory.erase(result.trajectory.begin());

  result.cost = graph_[target].distance;

  return result;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_IMPL_DFS_SOLVER_HPP
