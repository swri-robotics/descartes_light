#ifndef DESCARTES_LIGHT_SOLVERS_BGL_IMPL_BGL_DIJKSTRA_SOLVER_HPP
#define DESCARTES_LIGHT_SOLVERS_BGL_IMPL_BGL_DIJKSTRA_SOLVER_HPP

#include <descartes_light/bgl/bgl_dijkstra_solver.h>
#include <descartes_light/bgl/impl/event_visitors.hpp>

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <boost/graph/dijkstra_shortest_paths.hpp>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
template <typename FloatType, typename Visitors>
static VertexDesc<FloatType> solveDijkstra(BGLGraph<FloatType>& graph,
                                           std::vector<VertexDesc<FloatType>>& predecessors,
                                           const VertexDesc<FloatType>& source,
                                           const Visitors& event_visitors,
                                           const std::vector<std::vector<VertexDesc<FloatType>>>& ladder_rungs)
{
  // Internal properties
  auto index_prop_map = boost::get(boost::vertex_index, graph);
  auto weight_prop_map = boost::get(boost::edge_weight, graph);
  auto color_prop_map = boost::get(&Vertex<FloatType>::color, graph);
  auto distance_prop_map = boost::get(&Vertex<FloatType>::distance, graph);

  predecessors.resize(boost::num_vertices(graph), std::numeric_limits<std::size_t>::max());

  typedef typename boost::property_map<BGLGraph<FloatType>, boost::vertex_index_t>::type IndexMap;
  typedef boost::iterator_property_map<typename std::vector<VertexDesc<FloatType>>::iterator, IndexMap> PredecessorMap;
  PredecessorMap predecessor_it_map = boost::make_iterator_property_map(predecessors.begin(), index_prop_map);

  auto visitor = boost::make_dijkstra_visitor(event_visitors);

  // Perform the search
  try
  {
    boost::dijkstra_shortest_paths(graph,
                                   source,
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

    // In the case that the visitor does not throw the target vertex descriptor, find the lowest cost vertex in last
    // rung of the ladder graph
    auto target = std::min_element(ladder_rungs.back().begin(),
                                   ladder_rungs.back().end(),
                                   [&](const VertexDesc<FloatType>& a, const VertexDesc<FloatType>& b) {
                                     return graph[a].distance < graph[b].distance;
                                   });

    // Check that the identified lowest cost vertex is valid and has a cost less than inf
    if (target != ladder_rungs.back().end() && graph[*target].distance < std::numeric_limits<FloatType>::max())
      throw *target;
  }
  catch (const VertexDesc<FloatType>& target)
  {
    return target;
  }

  // If the visitor never threw the vertex descriptor, there was an issue with the search
  throw std::runtime_error("Search failed to encounter vertex associated with the last waypoint in the trajectory");
}

template <typename FloatType, typename Visitors>
BGLDijkstraSVSESolver<FloatType, Visitors>::BGLDijkstraSVSESolver(Visitors event_visitors, unsigned num_threads)
  : BGLSolverBaseSVSE<FloatType>(num_threads), event_visitors_(std::move(event_visitors))
{
}

template <typename FloatType, typename Visitors>
SearchResult<FloatType> BGLDijkstraSVSESolver<FloatType, Visitors>::search()
{
  // Convenience aliases
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  const auto& source_ = BGLSolverBase<FloatType>::source_;
  auto& predecessors_ = BGLSolverBase<FloatType>::predecessors_;
  const auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;

  VertexDesc<FloatType> target =
      solveDijkstra<FloatType, Visitors>(graph_, predecessors_, source_, event_visitors_, ladder_rungs_);

  SearchResult<FloatType> result;

  // Reconstruct the path from the predecesor map; remove the artificial start state
  const auto vd_path = BGLSolverBase<FloatType>::reconstructPath(source_, target);
  result.trajectory = BGLSolverBase<FloatType>::toStates(vd_path);
  result.trajectory.erase(result.trajectory.begin());

  result.cost = graph_[target].distance;

  return result;
}

template <typename FloatType, typename Visitors>
BGLDijkstraSVDESolver<FloatType, Visitors>::BGLDijkstraSVDESolver(Visitors event_visitors, unsigned num_threads)
  : BGLSolverBaseSVDE<FloatType>(num_threads), event_visitors_(std::move(event_visitors))
{
}

template <typename FloatType, typename Visitors>
SearchResult<FloatType> BGLDijkstraSVDESolver<FloatType, Visitors>::search()
{
  // Convenience aliases
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  const auto& source_ = BGLSolverBase<FloatType>::source_;
  auto& predecessors_ = BGLSolverBase<FloatType>::predecessors_;
  const auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;

  // Create the dynamic edge adding event visitor
  const auto& edge_eval_ = BGLSolverBaseSVDE<FloatType>::edge_eval_;
  auto vis = std::make_pair(add_all_edges_dynamically<FloatType, boost::on_examine_vertex>(edge_eval_, ladder_rungs_),
                            event_visitors_);

  VertexDesc<FloatType> target = solveDijkstra(graph_, predecessors_, source_, vis, ladder_rungs_);

  SearchResult<FloatType> result;

  // Reconstruct the path from the predecesor map; remove the artificial start state
  const auto vd_path = BGLSolverBase<FloatType>::reconstructPath(source_, target);
  result.trajectory = BGLSolverBase<FloatType>::toStates(vd_path);
  result.trajectory.erase(result.trajectory.begin());

  result.cost = graph_[target].distance;

  return result;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_IMPL_BGL_DIJKSTRA_SOLVER_HPP
