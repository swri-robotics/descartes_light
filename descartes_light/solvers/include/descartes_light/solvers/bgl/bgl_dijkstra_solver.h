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

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_BGL_DIJKSTRA_SOLVER_H
