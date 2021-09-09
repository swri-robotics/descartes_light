#ifndef DESCARTES_LIGHT_SOLVERS_BGL_BGL_DIJKSTRA_SOLVER_H
#define DESCARTES_LIGHT_SOLVERS_BGL_BGL_DIJKSTRA_SOLVER_H

#include <descartes_light/bgl/bgl_solver.h>
#include <descartes_light/bgl/event_visitors.h>

namespace descartes_light
{
/**
 * @brief BGL solver implementation that constructs vertices and edges in the build function and uses Dijkstra's
 * algorithm with a specifiable visitor to search the graph
 */
template <typename FloatType, typename Visitors>
class BGLDijkstraSVSESolver : public BGLSolverBaseSVSE<FloatType>
{
public:
  BGLDijkstraSVSESolver(Visitors event_visitors, unsigned num_threads = std::thread::hardware_concurrency());
  SearchResult<FloatType> search() override;

protected:
  /** @brief Event visitors for custom behavior in the search */
  Visitors event_visitors_;
};

using BGLDijkstraSVSESolverF = BGLDijkstraSVSESolver<float, early_terminator<boost::on_examine_vertex>>;
using BGLDijkstraSVSESolverD = BGLDijkstraSVSESolver<double, early_terminator<boost::on_examine_vertex>>;

/**
 * @brief BGL solver implementation that constructs vertices build function and uses Dijkstra's
 * algorithm with an edge-adding visitor to search the graph
 * @details Note: the solver internally creates an event visitor for adding edges dynamically to the graph. Therefore,
 * this event visitor does not need to be specified as a template parameter
 */
template <typename FloatType, typename Visitors>
class BGLDijkstraSVDESolver : public BGLSolverBaseSVDE<FloatType>
{
public:
  BGLDijkstraSVDESolver(Visitors event_visitors, unsigned num_threads = std::thread::hardware_concurrency());
  SearchResult<FloatType> search() override;

protected:
  /** @brief Event visitors for custom behavior in the search */
  Visitors event_visitors_;
};

using BGLDijkstraSVDESolverF = BGLDijkstraSVDESolver<float, early_terminator<boost::on_examine_vertex>>;
using BGLDijkstraSVDESolverD = BGLDijkstraSVDESolver<double, early_terminator<boost::on_examine_vertex>>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_BGL_DIJKSTRA_SOLVER_H
