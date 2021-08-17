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
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <descartes_light/solvers/bgl/dfs_sort_ladder_graph_solver.h>
#include <descartes_light/types.h>

//#define UNUSED(x) (void)(x)


namespace descartes_light
{

template <typename FloatType>
class VertexCostVisitor : public boost::default_dijkstra_visitor
{
  public:
  VertexCostVisitor(std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
                std::vector<std::vector<VertexDesc<FloatType>>>& ladder_rungs,
                BGLGraph<FloatType>& eg)
                : eval_(edge_eval),
                  ladder_rungs_(ladder_rungs),
                  mutable_graph_(eg)
       {
       }

  void discover_vertex(VertexDesc<FloatType> u, BGLGraph<FloatType> g)
  {
    if (g[u].rung_idx >= 0)
    {
      std::size_t current_rung = g[u].rung_idx;
      std::size_t next_rung = current_rung + 1;


      if (next_rung < ladder_rungs_.size())
      {
        std::size_t to_indx = 0;
        FloatType cost;
        bool edge_added = false; //not needed, added for clarity
        while (!edge_added)
        {
          auto& to_sample = mutable_graph_[ladder_rungs_[next_rung][to_indx]];

          std::pair<bool, FloatType> results =
              eval_[static_cast<size_t>(current_rung)]->evaluate(*g[u].sample.state, *to_sample.sample.state);
          if (results.first)
          {
            edge_added = true;
            cost = results.second + to_sample.sample.cost;
            //Dummy node prevents this problem?
            if (g[u].rung_idx == 0)
              cost += g[u].sample.cost;
            VertexDesc<FloatType> target_vert = ladder_rungs_[next_rung][to_indx];
            boost::add_edge(u, target_vert, cost, mutable_graph_);
            return;
          }
          else
          {
            if (to_indx != ladder_rungs_[next_rung].size())
            {
               ++to_indx;
            }
            else
            {
              throw std::runtime_error("No valid connections between two rungs");
            } //end else
          } // end no edge else
        } //end while loop
      } // end body search
      // Depth First Component; stop if @ last rung
      else
      {
        throw u;
      }
    }
  }
private:
  std::vector<typename EdgeEvaluator<FloatType>::ConstPtr> eval_{ nullptr };
  std::vector<std::vector<VertexDesc<FloatType>>>& ladder_rungs_;
  BGLGraph<FloatType>& mutable_graph_{ nullptr };
};

template <typename FloatType>
SearchResult<FloatType> DFSSortLadderGraphSolver<FloatType>::search()
{
  SearchResult<FloatType> result;

  // Internal properties
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  const auto& source_ = BGLSolverBase<FloatType>::source_;
  auto& predecessor_map_ = BGLSolverBase<FloatType>::predecessor_map_;
  auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;
  auto& edge_eval_ = BGLSolverBaseSVDE<FloatType>::edge_eval_;

  auto index_prop_map = boost::get(boost::vertex_index, graph_);
  auto weight_prop_map = boost::get(boost::edge_weight, graph_);

  //sorting lambda
  auto sortRungSamples = [&] (const VertexDesc<FloatType> &v1, const VertexDesc<FloatType> &v2) -> bool
     {
        return graph_[v1].sample.cost < graph_[v2].sample.cost;
     };

  //Sort Rungs
  for (std::size_t r = 0; r < ladder_rungs_.size(); ++r)
  {
    std::sort(ladder_rungs_[r].begin(), ladder_rungs_[r].end(), sortRungSamples); //both rungs are now sorted
  }

  result.cost = std::numeric_limits<FloatType>::max();
  result.trajectory = {};

  std::map<VertexDesc<FloatType>, VertexDesc<FloatType>> predecessor_map;
  boost::associative_property_map<std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>> predecessor_prop_map(predecessor_map_);

  std::map<VertexDesc<FloatType>, double> distance_map;
  boost::associative_property_map<std::map<VertexDesc<FloatType>, double>> distance_prop_map(distance_map);

  auto color_prop_map = boost::get(&Vertex<FloatType>::color, graph_);

  descartes_light::VertexCostVisitor<FloatType> visitor(edge_eval_, ladder_rungs_, graph_);


  try
  {
    boost::dijkstra_shortest_paths(graph_,
                                   source_,
                                   predecessor_prop_map,
                                   distance_prop_map,
                                   weight_prop_map,
                                   index_prop_map,
                                   std::less<>(),
                                   std::plus<>(),
                                   std::numeric_limits<double>::max(),
                                   0.0,
                                   visitor,
                                   color_prop_map);

  }
  catch (const VertexDesc<FloatType>& target)
  {
    const auto valid_path = BGLSolverBase<FloatType>::reconstructPath(source_, target);
    result.trajectory = BGLSolverBase<FloatType>::toStates(valid_path);

    // remove empty start state
    result.trajectory.erase(result.trajectory.begin());

    result.cost = distance_map.at(target);

    return result;
  }

  throw std::runtime_error("Failed to reach last rung");

}
} //end namespace
#endif //DESCARTES_LIGHT_DFS_SORT_LADDER_GRAPH_SOLVER_HPP
