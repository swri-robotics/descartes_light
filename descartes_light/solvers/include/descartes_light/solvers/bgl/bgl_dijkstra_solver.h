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
class BGLDijkstraSolverVE : public BGLSolverBaseVE<FloatType>
{
public:
  using BGLSolverBaseVE<FloatType>::BGLSolverBaseVE;

  SearchResult<FloatType> search() override;
};

using BGLDijkstraSolverVEF = BGLDijkstraSolverVE<float>;
using BGLDijkstraSolverVED = BGLDijkstraSolverVE<double>;

/**
 * @brief BGL solver implementation that constructs vertices and edges in the build function and uses Dijkstra's
 * algorithm with a visitor that terminates the search once a vertex in the last rung of the graph is encountered rather
 * than allowing it to continue until the distance to all nodes in the graph has been calculated
 */
template <typename FloatType>
class BGLEfficientDijkstraSolverVE : public BGLSolverBaseVE<FloatType>
{
public:
  using BGLSolverBaseVE<FloatType>::BGLSolverBaseVE;

  SearchResult<FloatType> search() override;
};

using BGLEfficientDijkstraSolverVEF = BGLEfficientDijkstraSolverVE<float>;
using BGLEfficientDijkstraSolverVED = BGLEfficientDijkstraSolverVE<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_BGL_DIJKSTRA_SOLVER_H
