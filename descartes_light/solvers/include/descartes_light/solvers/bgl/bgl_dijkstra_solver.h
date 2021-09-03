#ifndef DESCARTES_LIGHT_SOLVERS_BGL_BGL_DIJKSTRA_SOLVER_H
#define DESCARTES_LIGHT_SOLVERS_BGL_BGL_DIJKSTRA_SOLVER_H

#include <descartes_light/solvers/bgl/bgl_solver.h>
#include <descartes_light/solvers/bgl/event_visitors.h>

namespace descartes_light
{
/**
 * @brief BGL solver implementation that constructs vertices and edges in the build function and uses Dijkstra's
 * algorithm with a default visitor to search the graph
 */
template <typename FloatType, typename Visitors>
class BGLDijkstraSVSESolver : public BGLSolverBaseSVSE<FloatType, Visitors>
{
public:
  using BGLSolverBaseSVSE<FloatType, Visitors>::BGLSolverBaseSVSE;

  SearchResult<FloatType> search() override;
};

using BGLDijkstraSVSESolverF = BGLDijkstraSVSESolver<float, early_terminator<boost::on_examine_vertex>>;
using BGLDijkstraSVSESolverD = BGLDijkstraSVSESolver<double, early_terminator<boost::on_examine_vertex>>;

/**
 * @brief BGL solver implementation that constructs vertices build function and uses Dijkstra's
 * algorithm with an edge-adding visitor to search the graph
 */
template <typename FloatType, typename Visitors>
class BGLDijkstraSVDESolver : public BGLSolverBaseSVDE<FloatType, Visitors>
{
public:
  using BGLSolverBaseSVDE<FloatType, Visitors>::BGLSolverBaseSVDE;

  SearchResult<FloatType> search() override;
};

using BGLDijkstraSVDESolverF = BGLDijkstraSVDESolver<float, early_terminator<boost::on_examine_vertex>>;
using BGLDijkstraSVDESolverD = BGLDijkstraSVDESolver<double, early_terminator<boost::on_examine_vertex>>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_BGL_DIJKSTRA_SOLVER_H
