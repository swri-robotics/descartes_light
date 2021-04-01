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
#ifndef DESCARTES_LIGHT_IMPL_DESCARTES_LIGHT_HPP
#define DESCARTES_LIGHT_IMPL_DESCARTES_LIGHT_HPP

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <sstream>
#include <algorithm>
#include <Eigen/Geometry>
#include <chrono>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/descartes_light.h>
#include <descartes_light/ladder_graph_dag_search.h>

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
Solver<FloatType>::Solver(const std::size_t dof) : graph_{ dof }
{
}

template <typename FloatType>
void Solver<FloatType>::build(const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
                              const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
                              int num_threads)
{
  graph_.resize(trajectory.size());
  failed_vertices_.clear();
  failed_edges_.clear();

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
      Rung<FloatType>& r = graph_.getRung(static_cast<size_t>(i));
      r.nodes.reserve(vertex_data.size());
      for (const auto& v : vertex_data)
      {
        r.nodes.push_back(Node<FloatType>(v));
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
        std::pair<bool, FloatType> results =
            edge_eval[static_cast<size_t>(i - 1)]->evaluate(from.nodes[j].state, to.nodes[k].state);
        if (results.first)
        {
          found = true;
          from.nodes[j].edges.emplace_back(results.second, k);
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
    throw std::runtime_error("Failed to build graph.");
}

template <typename FloatType>
void Solver<FloatType>::build(const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
                              typename EdgeEvaluator<FloatType>::ConstPtr edge_eval,
                              int num_threads)
{
  std::vector<typename EdgeEvaluator<FloatType>::ConstPtr> evaluators(trajectory.size() - 1, edge_eval);
  build(trajectory, evaluators, num_threads);
}

template <typename FloatType>
std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> Solver<FloatType>::search()
{
  DAGSearch<FloatType> s(graph_);
  const auto cost = s.run();

  std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> solution;
  if (cost == std::numeric_limits<FloatType>::max())
    return solution;

  using Clock = std::chrono::high_resolution_clock;
  std::chrono::time_point<Clock> start_time = Clock::now();
  const auto indices = s.shortestPath();
  for (std::size_t i = 0; i < indices.size(); ++i)
    solution.push_back(graph_.getRung(i).nodes[indices[i]].state);

  double duration = std::chrono::duration<double>(Clock::now() - start_time).count();
  CONSOLE_BRIDGE_logDebug("Descartes took %0.4f seconds to search graph for solution with cost %0.4f.", duration, cost);
  return solution;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_IMPL_DESCARTES_LIGHT_HPP
