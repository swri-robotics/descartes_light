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
#ifndef DESCARTES_LIGHT_DFS_SORT_LADDER_GRAPH_SOLVER_HPP
#define DESCARTES_LIGHT_DFS_SORT_LADDER_GRAPH_SOLVER_HPP

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <omp.h>

#include <descartes_light/solvers/bgl/dfs_add_all_solver.h>
#include <descartes_light/solvers/bgl/impl/bgl_visitors.hpp>
#include <descartes_light/types.h>

#define UNUSED(x) (void)(x)

static void reportFailedVertices(const std::vector<std::size_t>& indices)
{
  if (indices.empty())
    CONSOLE_BRIDGE_logInform("No failed vertices");
  else
  {
    std::stringstream ss;
    ss << "Failed vertices:\n";
    for (const auto& i : indices)
      ss << "\t" << i << "\n";

    CONSOLE_BRIDGE_logWarn(ss.str().c_str());
  }
}

namespace descartes_light
{
template <typename FloatType>
DFSAddAllSolver<FloatType>::DFSAddAllSolver(const std::size_t dof, int num_threads)
  : dof_{dof}, num_threads_{ num_threads }
{
};

template <typename FloatType>
BuildStatus DFSAddAllSolver<FloatType>::buildImpl(
    const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
    const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_evaluators,
    const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_evaluators)
{
  BuildStatus status;
  edge_eval = std::move(edge_evaluators);

  // Build Vertices
  long num_waypoints = static_cast<long>(trajectory.size());
  ladder_rungs_.resize(num_waypoints);
  long cnt = 0;

  using Clock = std::chrono::high_resolution_clock;
  std::chrono::time_point<Clock> start_time = Clock::now();
  #pragma omp parallel for num_threads(num_threads_)
  for (long i = 0; i < static_cast<long>(trajectory.size()); ++i)
  {
    std::vector<StateSample<FloatType>> samples = trajectory[static_cast<size_t>(i)]->sample();
    if (!samples.empty())
    {
      for (StateSample<FloatType>& sample : samples)
      {
        VertexDesc<FloatType> vd;
        if (state_evaluators.empty())
        {
           throw std::runtime_error("Failed to find evaluate state");
        }
        else
        {
          std::pair<bool, FloatType> results = state_evaluators[static_cast<size_t>(i)]->evaluate(*samples[i].state);
          if (results.first)
          {
            sample.cost += results.second;
            vd = add_vertex(graph_);
            graph_[vd].sample = sample;
            graph_[vd].rung_idx = static_cast<long>(i);
            ladder_rungs_[static_cast<size_t>(i)].push_back(vd);
          }
        }
      }
    }
    else
    {
#pragma omp critical
      {
        status.failed_vertices.push_back(static_cast<size_t>(i));
      }
    }
#ifndef NDEBUG
#pragma omp critical
    {
      ++cnt;
      std::stringstream ss;
      ss << "Descartes Processed: " << cnt << " of " << num_waypoints << " vertices";
      CONSOLE_BRIDGE_logInform(ss.str().c_str());
    }
#endif
  }
  double duration = std::chrono::duration<double>(Clock::now() - start_time).count();
  CONSOLE_BRIDGE_logDebug("Descartes took %0.4f seconds to build vertices.", duration);

  // Create a zero-value, zero-cost start node and connect it with a zero-cost edge to each node in the first rung
  {
    source_ = boost::add_vertex(graph_);
    auto arr = std::make_shared<State<FloatType>>();
    graph_[source_].sample = StateSample<FloatType>{ arr, static_cast<FloatType>(0.0) };
    graph_[source_].rung_idx = -1;
    for (const VertexDesc<FloatType>& target : ladder_rungs_[0])
    {
      boost::add_edge(source_, target, static_cast<FloatType>(0.0), graph_);
    }
  }

  std::sort(status.failed_vertices.begin(), status.failed_vertices.end());
  reportFailedVertices(status.failed_vertices);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("LadderGraphSolver failed to build graph.");
  }

  return status;
}


template <typename FloatType>
std::vector<VertexDesc<FloatType>> DFSAddAllSolver<FloatType>::reconstructPath(const VertexDesc<FloatType>& source,
                                                                                             const VertexDesc<FloatType>& target) const
{
  // Reconstruct the path from predecessors
  std::vector<VertexDesc<FloatType>> path;

  VertexDesc<FloatType> v = target;
  path.push_back(v);

  for (VertexDesc<FloatType> u = predecessor_map_.at(v); u != v; v = u, u = predecessor_map_.at(v))
  {
    path.push_back(u);
  }
  std::reverse(path.begin(), path.end());

  // Check that the last traversed vertex is the source vertex
  if (v != source)
    throw std::runtime_error("Failed to find path through the graph");

  return path;
}

template <typename FloatType>
std::vector<typename State<FloatType>::ConstPtr>
DFSAddAllSolver<FloatType>::toStates(const std::vector<VertexDesc<FloatType>>& path) const
{
  // Get the state information from the graph using the vertex descriptors
  std::vector<typename State<FloatType>::ConstPtr> out;
  out.reserve(path.size());
  std::transform(path.begin(), path.end(), std::back_inserter(out), [this](const VertexDesc<FloatType>& vd) {
    return graph_[vd].sample.state;
  });

  return out;
}

template <typename FloatType>
SearchResult<FloatType> DFSAddAllSolver<FloatType>::search()
{
  SearchResult<FloatType> result;

  // Internal properties
  auto index_prop_map = boost::get(boost::vertex_index, graph_);
  auto weight_prop_map = boost::get(boost::edge_weight, graph_);

  result.cost = std::numeric_limits<FloatType>::max();
  result.trajectory = {};

  boost::associative_property_map<std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>> predecessor_prop_map(predecessor_map_);

  std::map<VertexDesc<FloatType>, double> distance_map;
  boost::associative_property_map<std::map<VertexDesc<FloatType>, double>> distance_prop_map(distance_map);

  boost::dijkstra_shortest_paths(graph_, source_, predecessor_prop_map, distance_prop_map, weight_prop_map,
                                 index_prop_map, std::less<>(), std::plus<>(), std::numeric_limits<double>::max(), 0.0,
                                 descartes_light::AddAllVisitor<FloatType>(edge_eval, predecessor_map_, ladder_rungs_, graph_));


  // Find lowest cost node in last rung
  auto target = std::min_element(ladder_rungs_.back().begin(), ladder_rungs_.back().end(), [&distance_map](const VertexDesc<FloatType>& a, const VertexDesc<FloatType>& b){
    return distance_map.at(a) < distance_map.at(b);
  });

  result.trajectory = toStates(reconstructPath(source_, *target));

  // remove empty start state
  result.trajectory.erase(result.trajectory.begin());

  result.cost = distance_map.at(*target);

  if(result.trajectory.empty())
    throw std::runtime_error("failed");

  return result;
}
}
#endif //DESCARTES_LIGHT_DFS_SORT_LADDER_GRAPH_SOLVER_HPP
