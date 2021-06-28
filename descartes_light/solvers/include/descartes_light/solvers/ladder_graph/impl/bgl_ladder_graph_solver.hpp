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
#ifndef DESCARTES_LIGHT_BGL_LADDER_GRAPH_SOLVER_HPP
#define DESCARTES_LIGHT_BGL_LADDER_GRAPH_SOLVER_HPP

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <omp.h>

#include <console_bridge/console.h>

//Boost::Graph will be used instead of ladder_graph.hpp
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

//probably included elsewhere
#include <descartes_light/solvers/ladder_graph/bgl_ladder_graph_solver.h>
#include <descartes_light/types.h>

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
//ToDo: Remove dof from here since it is unused? Or use it? Seems useful to know the len of a sample
template <typename FloatType>
BGLLadderGraphSolver<FloatType>::BGLLadderGraphSolver(const std::size_t dof, int num_threads)
  : num_threads_{ num_threads }
{
};

template <typename FloatType>
BuildStatus BGLLadderGraphSolver<FloatType>::buildImpl(
    const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
    const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_evaluators,
    const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_evaluators)
{
  BuildStatus status;
  //rough estimate of preallocation based on nodes/rung

  // Build Vertices
  long num_waypoints = static_cast<long>(trajectory.size());
  ladder_rungs.resize(num_waypoints);
  long cnt = 0;

  using Clock = std::chrono::high_resolution_clock;
  std::chrono::time_point<Clock> start_time = Clock::now();
#pragma omp parallel for num_threads(num_threads_)
  for (long i = 0; i < static_cast<long>(trajectory.size()); ++i)
  {
    std::vector<StateSample<FloatType>> samples = trajectory[static_cast<size_t>(i)]->sample();
    if (!samples.empty())
    {
      for (std::size_t sample_i = 0; sample_i <samples.size(); ++sample_i)
      {
        VertexDesc<FloatType> vd;
        if (state_evaluators.empty())
        {
          vd = add_vertex(samples[sample_i], graph_);
          ladder_rungs[i].push_back(vd);
        }
        else
        {
          std::pair<bool, FloatType> results = state_evaluators[static_cast<size_t>(i)]->evaluate(samples[i].state);
          if (results.first)
          {
            samples[i].cost += results.second;
            vd = add_vertex(samples[sample_i], graph_);
            ladder_rungs[i].push_back(vd);
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
    auto& from = ladder_rungs[i-1];
    const auto& to = ladder_rungs[i];

    bool found = false;
    for (std::size_t j = 0; j < from.size(); ++j)
    {
       StateSample<FloatType> from_sample = graph_[from[j]];
      for (std::size_t k = 0; k < to.size(); ++k)
      {
        // Consider the edge:
         StateSample<FloatType> to_sample = graph_[to[k]];
        std::pair<bool, FloatType> results =
            edge_evaluators[static_cast<size_t>(i - 1)]->evaluate(from_sample.state, to_sample.state);
        if (results.first)
        {
          found = true;
          if (i == 0)
          {
            //first edge captures first rung weights
            boost::add_edge (from[j] ,to[k], from_sample.cost + results.second + to_sample.cost, graph_);
          }
          else
          {
            boost::add_edge (from[j] ,to[k], results.second + to_sample.cost, graph_);
          }
        }
      }
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

  //todo: move these to a utilites function
  reportFailedVertices(status.failed_vertices);
  reportFailedEdges(status.failed_edges);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("LadderGraphSolver failed to build graph.");
  }

  return status;
}
}
#endif //DESCARTES_LIGHT_BGL_LADDER_GRAPH_SOLVER_HPP
