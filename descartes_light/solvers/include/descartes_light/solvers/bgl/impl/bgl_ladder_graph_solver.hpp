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
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <console_bridge/console.h>
#include <fstream>
#include <omp.h>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/solvers/bgl/bgl_ladder_graph_solver.h>
#include <descartes_light/solvers/bgl/utils.h>
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
template <typename FloatType>
BGLLadderGraphSolver<FloatType>::BGLLadderGraphSolver(unsigned num_threads) : num_threads_{ num_threads } {};

template <typename FloatType>
BuildStatus BGLLadderGraphSolver<FloatType>::buildImpl(
    const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
    const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_evaluators,
    const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_evaluators)
{
  BuildStatus status;

  // Build Vertices
  ladder_rungs_.resize(trajectory.size());
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

  // Build Edges
  cnt = 0;
  start_time = Clock::now();
#pragma omp parallel for num_threads(num_threads_)
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
  duration = std::chrono::duration<double>(Clock::now() - start_time).count();
  CONSOLE_BRIDGE_logDebug("Descartes took %0.4f seconds to build edges.", duration);

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
  std::sort(status.failed_edges.begin(), status.failed_edges.end());

  // todo: move these to a utilites function
  reportFailedVertices(status.failed_vertices);
  reportFailedEdges(status.failed_edges);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("LadderGraphSolver failed to build graph.");
  }

  return status;
}

template <typename FloatType>
std::vector<VertexDesc<FloatType>>
BGLLadderGraphSolver<FloatType>::reconstructPath(const VertexDesc<FloatType>& source,
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
BGLLadderGraphSolver<FloatType>::toStates(const std::vector<VertexDesc<FloatType>>& path) const
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
SearchResult<FloatType> BGLLadderGraphSolver<FloatType>::search()
{
  // Internal properties
  auto index_prop_map = boost::get(boost::vertex_index, graph_);
  auto weight_prop_map = boost::get(boost::edge_weight, graph_);
  auto color_prop_map = boost::get(&Vertex<FloatType>::color, graph_);
  auto distance_prop_map = boost::get(&Vertex<FloatType>::distance, graph_);

  predecessor_map_.clear();
  boost::associative_property_map<std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>> predecessor_prop_map(
      predecessor_map_);

  // Perform the search
  boost::dijkstra_shortest_paths(graph_,
                                 source_,
                                 predecessor_prop_map,
                                 distance_prop_map,
                                 weight_prop_map,
                                 index_prop_map,
                                 std::less<>(),
                                 std::plus<>(),
                                 std::numeric_limits<FloatType>::max(),
                                 static_cast<FloatType>(0.0),
                                 boost::default_dijkstra_visitor(),
                                 color_prop_map);

  // Find lowest cost node in last rung
  auto target = std::min_element(ladder_rungs_.back().begin(),
                                 ladder_rungs_.back().end(),
                                 [this](const VertexDesc<FloatType>& a, const VertexDesc<FloatType>& b) {
                                   return graph_[a].distance < graph_[b].distance;
                                 });

  SearchResult<FloatType> result;

  // Reconstruct the path from the predecesor map; remove the artificial start state
  result.trajectory = toStates(reconstructPath(source_, *target));
  result.trajectory.erase(result.trajectory.begin());

  result.cost = graph_[*target].distance;

  return result;
}

template <typename FloatType>
void BGLLadderGraphSolver<FloatType>::writeGraphWithPath(const std::string& filename) const
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

}  // namespace descartes_light
