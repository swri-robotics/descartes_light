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
//#include <iterator>
#include <omp.h>

#include <descartes_light/solvers/bgl/dfs_sort_ladder_graph_solver.h>
#include <descartes_light/types.h>

#define UNUSED(x) (void)(x)

//these should be put into a utils file
//static void reportFailedEdges(const std::vector<std::size_t>& indices)
//{
//  if (indices.empty())
//    CONSOLE_BRIDGE_logInform("No failed edges");
//  else
//  {
//    std::stringstream ss;
//    ss << "Failed edges:\n";
//    for (const auto& i : indices)
//      ss << "\t" << i << "\n";

//    CONSOLE_BRIDGE_logWarn(ss.str().c_str());
//  }
//}

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
DFSSortLadderGraphSolver<FloatType>::DFSSortLadderGraphSolver(const std::size_t dof, int num_threads)
  : dof_{dof}, num_threads_{ num_threads }
{
};

template <typename FloatType>
BuildStatus DFSSortLadderGraphSolver<FloatType>::buildImpl(
    const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
    const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_evaluators,
    const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_evaluators)
{
  BuildStatus status;
  auto edge_eval = std::move(edge_evaluators);

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
            vd = add_vertex(sample, graph_);
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

  std::sort(status.failed_vertices.begin(), status.failed_vertices.end());
  reportFailedVertices(status.failed_vertices);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("LadderGraphSolver failed to build graph.");
  }

  return status;
}


template <typename FloatType>
std::vector<typename State<FloatType>::ConstPtr> DFSSortLadderGraphSolver<FloatType>::reconstructPath(const VertexDesc<FloatType>& source, const VertexDesc<FloatType>& target,
                                        const std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>& predecessor_map)
{
  // Reconstruct the path from predecessors
  std::vector<typename State<FloatType>::ConstPtr> path;

  VertexDesc<FloatType> v = target;
  path.push_back(graph_[v].state);

  for (VertexDesc<FloatType> u = predecessor_map.at(v); u != v; v = u, u = predecessor_map.at(v))
  {
    path.push_back(graph_[u].state);
  }
  std::reverse(path.begin(), path.end());

  // Check that the last traversed vertex is the source vertex
  if (v != source)
    throw std::runtime_error("Failed to find path through the graph");

  return path;
}


template <typename FloatType>
SearchResult<FloatType> DFSSortLadderGraphSolver<FloatType>::search()
{
  SearchResult<FloatType> result;

  // use std::sample st there are no repeats whle trying add edges
  // ^ This may require constructing a sampler for each rung
  // we will want to store the ladder rung iterators for each rung to avoid repeats

  //sorting lambda

  auto sortRungSamples = [&] (const VertexDesc<FloatType> &v1, const VertexDesc<FloatType> &v2) -> bool
     {
        return graph_[v1].cost < graph_[v2].cost;
     };

  // while: time < time_constriant
  //sort first rung outside the loop
  std::sort(ladder_rungs[0].begin(), ladder_rungs[0].end(), sortRungSamples);

  //Depth first search
  for (std::size_t r = 1; r < ladder_rungs.size(); ++r)
  {
    std::sort(ladder_rungs[r].begin(), ladder_rungs[r].end(), sortRungSamples); //both rungs are now sorted

    bool edge_generated = false;

    std::size_t from_indx = 0;
    std::size_t to_indx = 0;
    FloatType cost;

    while (!edge_generated && from_indx != ladder_rungs[r-1].size())
    {
      StateSample<FloatType> from_sample = graph_[ladder_rungs[r-1][from_indx]];
      StateSample<FloatType> to_sample   = graph_[ladder_rungs[r][to_indx]];

      std::pair<bool, FloatType> results =
          edge_eval[static_cast<size_t>(r - 1)].evaluate(*from_sample.state, *to_sample.state);
      if (results.first)
      {
        cost = results.second + to_sample.cost;
        if (r-1 < 1)
          cost += from_sample.cost;
        boost::add_edge(ladder_rungs[r-1][from_indx], ladder_rungs[r][to_indx], cost, graph_);
        edge_generated = true;
      }
      else
      {
        if (to_indx != ladder_rungs[r].size())
        {
           ++to_indx;
        }
        else
        {
          to_indx = 0;
          ++from_indx;
        }
      } // end no edge else
    } // end edge gen while
    if (!edge_generated)
    {
      //throw std::runtime_error("Failed to find path through the graph");
      throw std::runtime_error("No valid connections between two rungs");
    }
  }

  // while time < time_limit && cost < cost limit
  //continue random sampling

  //end chrono while

  // finish graph construction, run dijkstra? or custom visitor search?

  // Internal properties
  auto index_prop_map = boost::get(boost::vertex_index, graph_);
  auto weight_prop_map = boost::get(boost::edge_weight, graph_);

  result.cost = std::numeric_limits<FloatType>::max();
  result.trajectory = {};

  //create a zero value, zero cost start sample
  auto arr = std::make_shared<State<FloatType>>(Eigen::Matrix<FloatType, Eigen::Dynamic, 1>::Zero(this->dof_));
  StateSample<FloatType> start_sample = StateSample<FloatType>{ arr, 0.0 };
  VertexDesc<FloatType> sd = add_vertex(start_sample, graph_);
  for (const VertexDesc<FloatType>& source_d : ladder_rungs[0])
  {
    boost::add_edge(sd, source_d, 0.0, graph_);
  }

  std::map<VertexDesc<FloatType>, VertexDesc<FloatType>> predecessor_map;
  boost::associative_property_map<std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>> predecessor_prop_map(predecessor_map);

  std::map<VertexDesc<FloatType>, double> distance_map;
  boost::associative_property_map<std::map<VertexDesc<FloatType>, double>> distance_prop_map(distance_map);

  boost::dijkstra_shortest_paths(graph_, sd, predecessor_prop_map, distance_prop_map, weight_prop_map,
                                 index_prop_map, std::less<>(), std::plus<>(), std::numeric_limits<double>::max(), 0.0,
                                 boost::default_dijkstra_visitor());


  // Find lowest cost node in last rung
  auto target_d = std::min_element(ladder_rungs.back().begin(), ladder_rungs.back().end(), [&distance_map](const VertexDesc<FloatType>& a, const VertexDesc<FloatType>& b){
    return distance_map.at(a) < distance_map.at(b);
  });

  result.trajectory = reconstructPath(sd, *target_d, predecessor_map);

  // remove empty start state
  result.trajectory.erase(result.trajectory.begin());

  result.cost = distance_map.at(*target_d);

  if(result.trajectory.empty())
    throw std::runtime_error("failed");

  return result;
}
}
#endif //DESCARTES_LIGHT_DFS_SORT_LADDER_GRAPH_SOLVER_HPP
