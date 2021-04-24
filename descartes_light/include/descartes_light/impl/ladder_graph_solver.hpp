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
#ifndef DESCARTES_LIGHT_IMPL_LADDER_GRAPH_SOLVER_HPP
#define DESCARTES_LIGHT_IMPL_LADDER_GRAPH_SOLVER_HPP

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <sstream>
#include <algorithm>
#include <Eigen/Geometry>
#include <chrono>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/ladder_graph/ladder_graph_solver.h>
#include <descartes_light/ladder_graph/ladder_graph_dag_search.h>

#define UNUSED(x) (void)(x)

static void reportFailedEdges(const std::vector<std::size_t>& indices)
{
  if (indices.empty())
    CONSOLE_BRIDGE_logInform("No failed edges");
  else
  {
    std::stringstream ss;
    ss << "Failed edges:\n";
    for (const auto& i : indices)
      ss << "\t" << i << "\n";

    CONSOLE_BRIDGE_logWarn(ss.str().c_str());
  }
}

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
template <typename FloatType, template <typename, typename...> class ContainerType>
LadderGraphSolver<FloatType, ContainerType>::LadderGraphSolver(const std::size_t dof) : graph_{ dof }
{
}

template <typename FloatType, template <typename, typename...> class ContainerType>
bool LadderGraphSolver<FloatType, ContainerType>::build(
    const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
    const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
    const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_eval,
    int num_threads)
{
  graph_.resize(trajectory.size());
  failed_vertices_.clear();
  failed_edges_.clear();

  std::vector<typename EdgeEvaluator<FloatType>::ConstPtr> edge_evaluators;
  if (edge_eval.size() == 1)
  {
    edge_evaluators.resize(trajectory.size() - 1);
    std::fill(edge_evaluators.begin(), edge_evaluators.end(), edge_eval.front());
  }
  else
  {
    edge_evaluators = edge_eval;
  }

  std::vector<typename StateEvaluator<FloatType>::ConstPtr> state_evaluators;
  if (state_eval.size() == 1)
  {
    state_evaluators.resize(trajectory.size());
    std::fill(state_evaluators.begin(), state_evaluators.end(), state_eval.front());
  }
  else
  {
    state_evaluators = state_eval;
  }

  // Build Vertices
  long num_waypoints = static_cast<long>(trajectory.size());
  long cnt = 0;

  using Clock = std::chrono::high_resolution_clock;
  std::chrono::time_point<Clock> start_time = Clock::now();
#pragma omp parallel for num_threads(num_threads)
  for (long i = 0; i < static_cast<long>(trajectory.size()); ++i)
  {
    std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> vertex_data = trajectory[static_cast<size_t>(i)]->sample();
    if (!vertex_data.empty())
    {
      auto& r = graph_.getRung(static_cast<size_t>(i));
      //      r.nodes.reserve(vertex_data.size());
      for (const auto& v : vertex_data)
      {
        std::pair<bool, FloatType> results = state_evaluators[static_cast<size_t>(i)]->evaluate(v);
        if (results.first)
          r.nodes.push_back(Node<FloatType, ContainerType>(v, results.second));
      }
    }
    else
    {
#pragma omp critical
      {
        failed_vertices_.push_back(static_cast<size_t>(i));
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

  // Build Edges
  cnt = 0;
  start_time = Clock::now();
#pragma omp parallel for num_threads(num_threads)
  for (long i = 1; i < static_cast<long>(trajectory.size()); ++i)
  {
    auto& from = graph_.getRung(static_cast<size_t>(i) - static_cast<size_t>(1));
    const auto& to = graph_.getRung(static_cast<size_t>(i));

    bool found = false;
    for (std::size_t j = 0; j < from.nodes.size(); ++j)
    {
      for (std::size_t k = 0; k < to.nodes.size(); ++k)
      {
        // Consider the edge:
        auto& from_node = *std::next(from.nodes.begin(), static_cast<long>(j));
        const auto& to_node = *std::next(to.nodes.begin(), static_cast<long>(k));
        std::pair<bool, FloatType> results =
            edge_eval[static_cast<size_t>(i - 1)]->evaluate(from_node.state, to_node.state);
        if (results.first)
        {
          found = true;
          from_node.edges.emplace_back(results.second, k);
        }
      }
    }

    if (!found)
    {
#pragma omp critical
      {
        failed_edges_.push_back(static_cast<size_t>(i) - static_cast<size_t>(1));
      }
    }
#ifndef NDEBUG
#pragma omp critical
    {
      ++cnt;
      std::stringstream ss;
      ss << "Descartes Processed: " << cnt << " of " << (num_waypoints - 1) << " edges";
      CONSOLE_BRIDGE_logInform(ss.str().c_str());
    }
#endif
  }
  UNUSED(cnt);
  UNUSED(num_waypoints);
  duration = std::chrono::duration<double>(Clock::now() - start_time).count();
  CONSOLE_BRIDGE_logDebug("Descartes took %0.4f seconds to build edges.", duration);

  std::sort(failed_vertices_.begin(), failed_vertices_.end());
  std::sort(failed_edges_.begin(), failed_edges_.end());

  reportFailedVertices(failed_vertices_);
  reportFailedEdges(failed_edges_);

  if (!failed_edges_.empty() || !failed_vertices_.empty())
  {
    CONSOLE_BRIDGE_logError("Failed to build graph.");
    return false;
  }

  return true;
}

template <typename FloatType, template <typename, typename...> class ContainerType>
std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> LadderGraphSolver<FloatType, ContainerType>::search()
{
  DAGSearch<FloatType, ContainerType> s(graph_);
  const auto cost = s.run();

  std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> solution;
  if (cost == std::numeric_limits<FloatType>::max())
    return solution;

  using Clock = std::chrono::high_resolution_clock;
  std::chrono::time_point<Clock> start_time = Clock::now();
  const auto indices = s.shortestPath();
  for (std::size_t i = 0; i < indices.size(); ++i)
    solution.push_back(std::next(graph_.getRung(i).nodes.begin(), static_cast<long>(indices[i]))->state);

  double duration = std::chrono::duration<double>(Clock::now() - start_time).count();
  CONSOLE_BRIDGE_logDebug("Descartes took %0.4f seconds to search graph for solution with cost %0.4f.", duration, cost);
  return solution;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_IMPL_LADDER_GRAPH_SOLVER_HPP
