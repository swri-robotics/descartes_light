#ifndef DESCARTES_LIGHT_SOLVERS_BGL_IMPL_BGL_DIJKSTRA_SOLVER_HPP
#define DESCARTES_LIGHT_SOLVERS_BGL_IMPL_BGL_DIJKSTRA_SOLVER_HPP

#include <descartes_light/solvers/bgl/bgl_dijkstra_solver.h>
#include <descartes_light/solvers/bgl/impl/event_visitors.hpp>

#include <descartes_light/solvers/bgl/boost_graph_types.h>

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <boost/graph/dijkstra_shortest_paths.hpp>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
template <typename FloatType>
SearchResult<FloatType> BGLDijkstraSVSESolver<FloatType>::search()
{
  // Convenience aliases
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  const auto& source_ = BGLSolverBase<FloatType>::source_;
  auto& predecessor_map_ = BGLSolverBase<FloatType>::predecessor_map_;
  const auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;

  // Internal properties
  auto index_prop_map = boost::get(boost::vertex_index, graph_);
  auto weight_prop_map = boost::get(boost::edge_weight, graph_);
  auto color_prop_map = boost::get(&Vertex<FloatType>::color, graph_);
  auto distance_prop_map = boost::get(&Vertex<FloatType>::distance, graph_);

  typedef typename boost::property_map<BGLGraph<FloatType>, boost::vertex_index_t>::type IndexMap;
  typedef boost::iterator_property_map<typename std::vector<VertexDesc<FloatType>>::iterator, IndexMap> VertMap;
  predecessor_map_.resize(boost::num_vertices(graph_), std::numeric_limits<std::size_t>::max());
  VertMap predecessor_it_map = boost::make_iterator_property_map(predecessor_map_.begin(), index_prop_map);


  // Perform the search
  boost::dijkstra_shortest_paths(graph_,
                                 source_,
                                 predecessor_it_map,
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
                                 [&](const VertexDesc<FloatType>& a, const VertexDesc<FloatType>& b) {
                                   return graph_[a].distance < graph_[b].distance;
                                 });

  SearchResult<FloatType> result;

  // Reconstruct the path from the predecesor map; remove the artificial start state
  const auto vd_path = BGLSolverBase<FloatType>::reconstructPath(source_, *target);
  result.trajectory = BGLSolverBase<FloatType>::toStates(vd_path);
  result.trajectory.erase(result.trajectory.begin());

  result.cost = graph_[*target].distance;

  return result;
}

template <typename FloatType>
SearchResult<FloatType> BGLEfficientDijkstraSVSESolver<FloatType>::search()
{
  // Convenience aliases
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  const auto& source_ = BGLSolverBase<FloatType>::source_;
  auto& predecessor_map_ = BGLSolverBase<FloatType>::predecessor_map_;
  const auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;

  // Internal properties
  auto index_prop_map = boost::get(boost::vertex_index, graph_);
  auto weight_prop_map = boost::get(boost::edge_weight, graph_);
  auto color_prop_map = boost::get(&Vertex<FloatType>::color, graph_);
  auto distance_prop_map = boost::get(&Vertex<FloatType>::distance, graph_);

  typedef typename boost::property_map<BGLGraph<FloatType>, boost::vertex_index_t>::type IndexMap;
  typedef boost::iterator_property_map<typename std::vector<VertexDesc<FloatType>>::iterator, IndexMap> VertMap;
  predecessor_map_.resize(boost::num_vertices(graph_), std::numeric_limits<std::size_t>::max());
  VertMap predecessor_it_map = boost::make_iterator_property_map(predecessor_map_.begin(), index_prop_map);

  const long last_rung_idx = static_cast<long>(ladder_rungs_.size() - 1);
  auto visitor = boost::make_dijkstra_visitor(early_terminator<FloatType>(last_rung_idx));

  // Perform the search
  try
  {
    boost::dijkstra_shortest_paths(graph_,
                                   source_,
                                   predecessor_it_map,
                                   distance_prop_map,
                                   weight_prop_map,
                                   index_prop_map,
                                   std::less<>(),
                                   std::plus<>(),
                                   std::numeric_limits<FloatType>::max(),
                                   static_cast<FloatType>(0.0),
                                   visitor,
                                   color_prop_map);
  }
  catch (const VertexDesc<FloatType>& target)
  {
    SearchResult<FloatType> result;

    // Reconstruct the path from the predecesor map; remove the artificial start state
    const auto vd_path = BGLSolverBase<FloatType>::reconstructPath(source_, target);
    result.trajectory = BGLSolverBase<FloatType>::toStates(vd_path);
    result.trajectory.erase(result.trajectory.begin());

    result.cost = graph_[target].distance;

    return result;
  }

  // If the visitor never threw the vertex descriptor, there was an issue with the search
  throw std::runtime_error("Search failed to encounter vertex associated with the last waypoint in the trajectory");
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_IMPL_BGL_DIJKSTRA_SOLVER_HPP
