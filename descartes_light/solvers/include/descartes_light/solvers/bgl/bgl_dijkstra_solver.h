#ifndef DESCARTES_LIGHT_SOLVERS_BGL_BGL_DIJKSTRA_SOLVER_H
#define DESCARTES_LIGHT_SOLVERS_BGL_BGL_DIJKSTRA_SOLVER_H

#include <descartes_light/solvers/bgl/bgl_solver.h>

namespace descartes_light
{
/**
 * @brief BGL solver implementation that constructs vertices and edges in the build function and uses Dijkstra's
 * algorithm with a default visitor to search the graph
 */
template <typename FloatType>
class BGLDijkstraSVSESolver : public BGLSolverBaseSVSE<FloatType>
{
public:
  using BGLSolverBaseSVSE<FloatType>::BGLSolverBaseSVSE;

  SearchResult<FloatType> search() override;
};

using BGLDijkstraSVSESolverF = BGLDijkstraSVSESolver<float>;
using BGLDijkstraSVSESolverD = BGLDijkstraSVSESolver<double>;

/**
 * @brief BGL solver implementation that constructs vertices and edges in the build function and uses Dijkstra's
 * algorithm with a visitor that terminates the search once a vertex in the last rung of the graph is encountered rather
 * than allowing it to continue until the distance to all nodes in the graph has been calculated
 */
template <typename FloatType>
class BGLEfficientDijkstraSVSESolver : public BGLSolverBaseSVSE<FloatType>
{
public:
  using BGLSolverBaseSVSE<FloatType>::BGLSolverBaseSVSE;

  SearchResult<FloatType> search() override;
};

using BGLEfficientDijkstraSVSESolverF = BGLEfficientDijkstraSVSESolver<float>;
using BGLEfficientDijkstraSVSESolverD = BGLEfficientDijkstraSVSESolver<double>;

/**
 * @brief BGL solver implementation that constructs vertices build function and uses Dijkstra's
 * algorithm with an edge-adding visitor to search the graph
 */
template <typename FloatType>
class BGLDijkstraSVDESolver : public BGLSolverBaseSVDE<FloatType>
{
public:
  using BGLSolverBaseSVDE<FloatType>::BGLSolverBaseSVDE;

  SearchResult<FloatType> search() override;
};

using BGLDijkstraSVDESolverF = BGLDijkstraSVDESolver<float>;
using BGLDijkstraSVDESolverD = BGLDijkstraSVDESolver<double>;

/**
 * @brief BGL solver implementation that constructs vertices in the build function and uses Dijkstra's
 * algorithm with a visitor that adds edges and terminates the search once a vertex in the last rung of
 * the graph is encountered rather than allowing it to continue until the distance to all nodes in the
 * graph has been calculated
 */
template <typename FloatType>
class BGLEfficientDijkstraSVDESolver : public BGLSolverBaseSVDE<FloatType>
{
public:
  using BGLSolverBaseSVDE<FloatType>::BGLSolverBaseSVDE;

  SearchResult<FloatType> search() override;
};

using BGLEfficientDijkstraSVDESolverF = BGLEfficientDijkstraSVDESolver<float>;
using BGLEfficientDijkstraSVDESolverD = BGLEfficientDijkstraSVDESolver<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_BGL_DIJKSTRA_SOLVER_H
