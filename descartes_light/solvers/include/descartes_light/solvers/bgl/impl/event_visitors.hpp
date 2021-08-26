#ifndef DESCARTES_LIGHT_SOLVERS_BGL_IMPL_EVENT_VISITORS_HPP
#define DESCARTES_LIGHT_SOLVERS_BGL_IMPL_EVENT_VISITORS_HPP

#include <descartes_light/solvers/bgl/boost_graph_types.h>

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <boost/graph/visitors.hpp>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
/**
 * @brief Event visitor that terminates the search when a vertex in the last rung of the graph is encountered
 * @details Throws the vertex descriptor that is the termination of the path once a vertex in the last rung of
 * the graph is encountered
 */
template <typename FloatType>
struct early_terminator : public boost::base_visitor<early_terminator<FloatType>>
{
  /** @brief Event filter typedef defining the events for which this visitor can be used */
  typedef boost::on_examine_vertex event_filter;

  early_terminator(long last_rung_idx) : last_rung_idx_(last_rung_idx) {}

  void operator()(VertexDesc<FloatType> u, const BGLGraph<FloatType>& g)
  {
    if (g[u].rung_idx == last_rung_idx_)
      throw u;
  }

  const long last_rung_idx_;
};

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_IMPL_EVENT_VISITORS_HPP
