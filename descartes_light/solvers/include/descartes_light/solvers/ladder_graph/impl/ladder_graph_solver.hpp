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
#include <omp.h>
#include <console_bridge/console.h>
#include <sstream>
#include <algorithm>
#include <Eigen/Geometry>
#include <chrono>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph_dag_search.h>

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
template <typename FloatType>
LadderGraphSolver<FloatType>::LadderGraphSolver(std::size_t dof, int num_threads)
  : graph_{ dof }, num_threads_{ num_threads }
{
}

template <typename FloatType>
BuildStatus LadderGraphSolver<FloatType>::buildImpl(
    const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
    const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_evaluators,
    const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_evaluators)
{
  BuildStatus status;
  graph_.resize(trajectory.size());

  // Build Vertices
  long num_waypoints = static_cast<long>(trajectory.size());
  long cnt = 0;

  using Clock = std::chrono::high_resolution_clock;
  std::chrono::time_point<Clock> start_time = Clock::now();
#pragma omp parallel for num_threads(num_threads_)
  for (long i = 0; i < static_cast<long>(trajectory.size()); ++i)
  {
    std::vector<StateSample<FloatType>> samples = trajectory[static_cast<size_t>(i)]->sample();
    if (!samples.empty())
    {
      auto& r = graph_.getRung(static_cast<size_t>(i));
      r.nodes.reserve(samples.size());
      for (auto& sample : samples)
      {
        if (state_evaluators.empty())
        {
          r.nodes.push_back(Node<FloatType>(sample));
        }
        else
        {
          std::pair<bool, FloatType> results = state_evaluators[static_cast<size_t>(i)]->evaluate(*sample.state);
          if (results.first)
          {
            sample.cost += results.second;
            r.nodes.push_back(Node<FloatType>(sample));
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

  // Build Edges
  cnt = 0;
  start_time = Clock::now();
#pragma omp parallel for num_threads(num_threads_)
  for (long i = 1; i < static_cast<long>(trajectory.size()); ++i)
  {
    auto& from = graph_.getRung(static_cast<size_t>(i) - static_cast<size_t>(1));
    const auto& to = graph_.getRung(static_cast<size_t>(i));

    bool found = false;
    for (std::size_t j = 0; j < from.nodes.size(); ++j)
    {
      auto& from_node = from.nodes[j];
      for (std::size_t k = 0; k < to.nodes.size(); ++k)
      {
        // Consider the edge:
        const auto& to_node = to.nodes[k];
        std::pair<bool, FloatType> results =
            edge_evaluators[static_cast<size_t>(i - 1)]->evaluate(*from_node.sample.state, *to_node.sample.state);
        if (results.first)
        {
          found = true;
          from_node.edges.emplace_back(results.second, k);
        }
      }

      // Since we are using emplace_back (or push_back) it doubles the capacity everytime the
      // capacity is reached so this could be huge when solving large ladder graph problems.
      // So shrink the capacity to fit
      // @todo Should max possible size be reserved first
      from_node.edges.shrink_to_fit();
    }

    if (!found)
    {
#pragma omp critical
      {
        status.failed_edges.push_back(static_cast<size_t>(i) - static_cast<size_t>(1));
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

  std::sort(status.failed_vertices.begin(), status.failed_vertices.end());
  std::sort(status.failed_edges.begin(), status.failed_edges.end());

  reportFailedVertices(status.failed_vertices);
  reportFailedEdges(status.failed_edges);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("LadderGraphSolver failed to build graph.");
  }

  return status;
}

template <typename FloatType>
SearchResult<FloatType> LadderGraphSolver<FloatType>::search()
{
  using Clock = std::chrono::high_resolution_clock;
  std::chrono::time_point<Clock> start_time = Clock::now();

  DAGSearch<FloatType> s(graph_);
  const auto cost = s.run();

  double duration = std::chrono::duration<double>(Clock::now() - start_time).count();
  CONSOLE_BRIDGE_logDebug("Descartes took %0.4f seconds to search graph for solution with cost %0.4f.", duration, cost);

  if (std::abs(cost - std::numeric_limits<FloatType>::max()) < std::numeric_limits<FloatType>::epsilon())
    throw std::runtime_error("Failed to find path through the graph");

  SearchResult<FloatType> result;
  result.cost = cost;

  const auto indices = s.shortestPath();
  result.trajectory.reserve(indices.size());
  for (std::size_t i = 0; i < indices.size(); ++i)
    result.trajectory.push_back(graph_.getRung(i).nodes[indices[i]].sample.state);

  return result;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_IMPL_LADDER_GRAPH_SOLVER_HPP
