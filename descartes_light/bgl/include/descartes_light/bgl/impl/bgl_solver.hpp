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
#pragma once

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <fstream>
#include <omp.h>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/bgl/bgl_solver.h>
#include <descartes_light/bgl/utils.h>
#include <descartes_light/types.h>

#define UNUSED(x) (void)(x)

using Clock = std::chrono::high_resolution_clock;

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
BGLSolverBase<FloatType>::BGLSolverBase(unsigned num_threads) : num_threads_{ num_threads }
{
}

template <typename FloatType>
std::vector<VertexDesc<FloatType>> BGLSolverBase<FloatType>::reconstructPath(const VertexDesc<FloatType>& source,
                                                                             const VertexDesc<FloatType>& target) const
{
  // Reconstruct the path from predecessors
  std::vector<VertexDesc<FloatType>> path;

  VertexDesc<FloatType> v = target;
  path.push_back(v);

  for (VertexDesc<FloatType> u = predecessors_.at(v); u != v; v = u, u = predecessors_.at(v))
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
BGLSolverBase<FloatType>::toStates(const std::vector<VertexDesc<FloatType>>& path) const
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
void BGLSolverBase<FloatType>::writeGraphWithPath(const std::string& filename) const
{
  std::ofstream file(filename);
  if (!file.good())
    throw std::runtime_error("Failed to open file '" + filename + "'");

  SubGraph<FloatType> sg = createDecoratedSubGraph(graph_);

  // Get the path through the graph
  auto target = std::min_element(ladder_rungs_.back().begin(),
                                 ladder_rungs_.back().end(),
                                 [this](const VertexDesc<FloatType>& a, const VertexDesc<FloatType>& b) {
                                   return graph_[a].distance < graph_[b].distance;
                                 });
  const std::vector<VertexDesc<FloatType>> path = reconstructPath(source_, *target);

  // Colorize the path
  for (const VertexDesc<FloatType>& v : path)
  {
    boost::get(boost::vertex_attribute, sg)[v][FILLCOLOR_ATTR] = "green";
  }

  boost::write_graphviz(file, sg);
}

template <typename FloatType>
BuildStatus BGLSolverBaseSVDE<FloatType>::buildImpl(
    const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
    const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_evaluators,
    const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_evaluators)
{
  // Convenience aliases
  auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  auto& source_ = BGLSolverBase<FloatType>::source_;

  BuildStatus status;
  edge_eval_ = std::move(edge_evaluators);

  // Build Vertices
  ladder_rungs_.resize(trajectory.size());
  long cnt = 0;

  std::chrono::time_point<Clock> start_time = Clock::now();
#pragma omp parallel for num_threads(BGLSolverBase <FloatType>::num_threads_)
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
          vd = boost::add_vertex(graph_);
          graph_[vd].sample = sample;
          graph_[vd].rung_idx = static_cast<long>(i);
          ladder_rungs_[static_cast<size_t>(i)].push_back(vd);
        }
        else
        {
          std::pair<bool, FloatType> results = state_evaluators[static_cast<size_t>(i)]->evaluate(*sample.state);
          if (results.first)
          {
            sample.cost += results.second;
            vd = boost::add_vertex(graph_);
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
      ss << "Descartes Processed: " << cnt << " of " << trajectory.size() << " vertices";
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

  // Report the failed vertices
  std::sort(status.failed_vertices.begin(), status.failed_vertices.end());
  // todo: move these to a utilites function
  reportFailedVertices(status.failed_vertices);

  return status;
}

template <typename FloatType>
BuildStatus BGLSolverBaseSVSE<FloatType>::buildImpl(
    const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
    const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_evaluators,
    const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_evaluators)
{
  // Convenience aliases
  auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;
  auto& graph_ = BGLSolverBase<FloatType>::graph_;

  // Use the base class to build the vertices
  BuildStatus status = BGLSolverBaseSVDE<FloatType>::buildImpl(trajectory, edge_evaluators, state_evaluators);

  // Build Edges
  long cnt = 0;
  auto start_time = Clock::now();
#pragma omp parallel for num_threads(BGLSolverBase <FloatType>::num_threads_)
  for (long i = 1; i < static_cast<long>(trajectory.size()); ++i)
  {
    auto& from = ladder_rungs_[static_cast<size_t>(i) - 1];
    const auto& to = ladder_rungs_[static_cast<size_t>(i)];

    bool found = false;
    for (long j = 0; j < static_cast<long>(from.size()); ++j)
    {
      const StateSample<FloatType> from_sample = graph_[from[static_cast<size_t>(j)]].sample;
      for (long k = 0; k < static_cast<long>(to.size()); ++k)
      {
        // Consider the edge:
        const StateSample<FloatType> to_sample = graph_[to[static_cast<size_t>(k)]].sample;
        std::pair<bool, FloatType> results =
            edge_evaluators[static_cast<size_t>(i - 1)]->evaluate(*from_sample.state, *to_sample.state);
        if (results.first)
        {
          found = true;
          if (i == 1)
          {
            // first edge captures first rung weights
            boost::add_edge(from[static_cast<size_t>(j)],
                            to[static_cast<size_t>(k)],
                            from_sample.cost + results.second + to_sample.cost,
                            graph_);
          }
          else
          {
            boost::add_edge(
                from[static_cast<size_t>(j)], to[static_cast<size_t>(k)], results.second + to_sample.cost, graph_);
          }
        }
      }
    }  // node loop

    if (!found)
    {
#pragma omp critical
      {
        status.failed_edges.push_back(static_cast<size_t>(i) - 1);
      }
    }
#ifndef NDEBUG
#pragma omp critical
    {
      ++cnt;
      std::stringstream ss;
      ss << "Descartes Processed: " << cnt << " of " << (trajectory.size() - 1) << " edges";
      CONSOLE_BRIDGE_logInform(ss.str().c_str());
    }
#endif
  }  // rung loop
  UNUSED(cnt);
  auto duration = std::chrono::duration<double>(Clock::now() - start_time).count();
  CONSOLE_BRIDGE_logDebug("Descartes took %0.4f seconds to build edges.", duration);

  // Report the failed edges
  std::sort(status.failed_edges.begin(), status.failed_edges.end());
  reportFailedEdges(status.failed_edges);

  return status;
}

}  // namespace descartes_light
