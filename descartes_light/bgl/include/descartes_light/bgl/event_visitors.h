#ifndef DESCARTES_LIGHT_SOLVERS_BGL_EVENT_VISITORS
#define DESCARTES_LIGHT_SOLVERS_BGL_EVENT_VISITORS

#include <descartes_light/bgl/boost_graph_types.h>

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <boost/graph/visitors.hpp>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
/**
 * @brief Event visitor that terminates the search when a vertex in the last rung of the graph is examined
 * @details Throws the vertex descriptor that is the termination of the path once a vertex in the last rung of
 * the graph is operated on
 */
template <typename EventType>
struct early_terminator : public boost::base_visitor<early_terminator<EventType>>
{
  /** @brief Event filter typedef defining the events for which this visitor can be used */
  typedef EventType event_filter;

  early_terminator(long last_rung_idx);

  template <typename FloatType>
  void operator()(VertexDesc<FloatType> u, const BGLGraph<FloatType>& g);

  const long last_rung_idx_;
};

/**
 * @brief Event visitor that adds all edges to each vertex dynamically as the vertex is discovered during the graph
 * search
 */
template <typename FloatType, typename EventType>
struct add_all_edges_dynamically : public boost::base_visitor<add_all_edges_dynamically<FloatType, EventType>>
{
  /** @brief Event filter typedef defining the events for which this visitor can be used */
  typedef EventType event_filter;

  add_all_edges_dynamically(std::vector<typename EdgeEvaluator<FloatType>::ConstPtr> edge_eval,
                            std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs);

  void operator()(VertexDesc<FloatType> u, const BGLGraph<FloatType>& g);

  const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr> eval_;
  const std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs_;
};

/**
 * @brief Event visitor for updating vertex cost
 */
struct cost_recorder : public boost::base_visitor<cost_recorder>
{
  /** @brief Event filter typedef defining the events for which this visitor can be used */
  typedef boost::on_tree_edge event_filter;

  template <typename FloatType>
  void operator()(EdgeDesc<FloatType> e, const BGLGraph<FloatType>& g);
};

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_EVENT_VISITORS_H
