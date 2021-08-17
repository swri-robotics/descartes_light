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
#ifndef DESCARTES_LIGHT_SOLVERS_BGL_IMPL_ADD_ALL_DYNAMIC_DIJKSTRA_SOLVER_HPP
#define DESCARTES_LIGHT_SOLVERS_BGL_IMPL_ADD_ALL_DYNAMIC_DIJKSTRA_SOLVER_HPP

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <omp.h>
#include <boost/graph/depth_first_search.hpp>
#include <descartes_light/solvers/bgl/dfs_add_all_solver.h>
#include <descartes_light/types.h>

namespace descartes_light
{

template <typename FloatType>
class AddAllVisitor : public boost::default_dfs_visitor
{
  public:
  AddAllVisitor(std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
                std::vector<std::vector<VertexDesc<FloatType>>>& ladder_rungs,
                std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>& predecessor_map)
                : eval_(edge_eval),
                  ladder_rungs_(ladder_rungs),
                  preds_(predecessor_map)
       {
       }

  void discover_vertex(VertexDesc<FloatType> u, const BGLGraph<FloatType>& g)
  {
    int out_deg = boost::out_degree(u, g);
    // return if the vertex has any out edges
    if (out_deg == 0)
    {
      std::size_t current_rung = g[u].rung_idx;
      std::size_t next_rung = current_rung + 1;
      if (next_rung < ladder_rungs_.size())
      {
        FloatType cost;
        for (std::size_t s = 0; s < ladder_rungs_[next_rung].size(); ++s)
        {
          std::pair<bool, FloatType> results =
              eval_[static_cast<size_t>(current_rung)]->evaluate(*g[u].sample.state, *g[ladder_rungs_[next_rung][s]].sample.state);
          if (results.first)
          {
            cost = results.second + g[ladder_rungs_[next_rung][s]].sample.cost;
            if (current_rung == 0)
              cost += g[u].sample.cost;
            VertexDesc<FloatType> sap = ladder_rungs_[next_rung][s];
            BGLGraph<FloatType>* mutable_graph_ = const_cast<BGLGraph<FloatType>*>(&g);
            //edges are not persisting
            std::pair<EdgeDesc<FloatType>, bool> result2 = boost::add_edge(u, sap, cost, *mutable_graph_);
            if (result2.second)
                cost = 0;
          }
        }
      }
      // Depth First Component
      else
      {
        throw u;
      }
    }
    return;
  }

  void tree_edge(EdgeDesc<FloatType> e, const BGLGraph<FloatType>& g)
  {
    VertexDesc<FloatType> target = boost::target(e, g);
    VertexDesc<FloatType> source = boost::source(e, g);
    BGLGraph<FloatType>* mutable_graph_ = const_cast<BGLGraph<FloatType>*>(&g);

    mutable_graph_->operator[](target).distance = g[source].distance + g[target].sample.cost;
    // there exist a predecessor recorder event visitor already implemented in boost_visitors
    //preds_.insert( std::pair<VertexDesc<FloatType>, VertexDesc<FloatType>>(boost::target(e, g), boost::targetsource(e, g)) );
    preds_[boost::target(e, g)] =  boost::source(e, g);
   }

private:
  std::vector<typename EdgeEvaluator<FloatType>::ConstPtr> eval_{ nullptr };
  std::vector<std::vector<VertexDesc<FloatType>>>& ladder_rungs_;
  std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>& preds_;  
};

template <typename FloatType>
SearchResult<FloatType> DFSAddAllSolver<FloatType>::search()
{
  SearchResult<FloatType> result;

  // Internal properties
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  const auto& source_ = BGLSolverBase<FloatType>::source_;
  auto& predecessor_map_ = BGLSolverBase<FloatType>::predecessor_map_;
  auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;
  auto& edge_eval_ = BGLSolverBaseSVDE<FloatType>::edge_eval_;

  result.cost = std::numeric_limits<FloatType>::max();
  result.trajectory = {};

  //boost::associative_property_map<std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>> predecessor_prop_map(predecessor_map_);

  auto color_prop_map = boost::get(&Vertex<FloatType>::color, graph_);

  descartes_light::AddAllVisitor<FloatType> vis(edge_eval_, ladder_rungs_, predecessor_map_);

  try
  {
      boost::depth_first_search(graph_,
                                vis,
                                color_prop_map,
                                source_);
  }
  catch (const VertexDesc<FloatType>& target)
  {
    predecessor_map_.insert( std::pair<VertexDesc<FloatType>, VertexDesc<FloatType>>(source_,source_));
    const auto valid_path = BGLSolverBase<FloatType>::reconstructPath(source_, target);
    result.trajectory = BGLSolverBase<FloatType>::toStates(valid_path);

    // remove empty start state
    result.trajectory.erase(result.trajectory.begin());
    result.cost = graph_[target].distance;
    //result.cost = distance_map.at(target);

    return result;
  }

  throw std::runtime_error("Failed to reach last rung");

}

}  //namespace descartes_light

#endif //DESCARTES_LIGHT_SOLVERS_BGL_IMPL_ADD_ALL_DYNAMIC_DIJKSTRA_SOLVER_HPP
