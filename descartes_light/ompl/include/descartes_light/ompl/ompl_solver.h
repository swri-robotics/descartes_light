#ifndef DESCARTES_LIGHT_SOLVERS_OMPL_OMPL_SOLVER_H
#define DESCARTES_LIGHT_SOLVERS_OMPL_OMPL_SOLVER_H

#include <descartes_light/bgl/bgl_solver.h>

namespace descartes_light
{
/**
 * @brief BGL solver implementation that constructs vertices and edges in the build function and uses Dijkstra's
 * algorithm with a default visitor to search the graph
 */
template <typename FloatType>
class BGLOMPLSVDESolver : public BGLSolverBaseSVDE<FloatType>
{
public:
  using BGLSolverBaseSVDE<FloatType>::BGLSolverBaseSVDE;

  SearchResult<FloatType> search() override;

protected:
  const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr> edge_eval_;
};

using BGLOMPLSVDESolverF = BGLOMPLSVDESolver<float>;
using BGLOMPLSVDESolverD = BGLOMPLSVDESolver<double>;

/**
 * @brief BGL solver implementation that constructs vertices and edges in the build function and uses Dijkstra's
 * algorithm with a visitor that terminates the search once a vertex in the last rung of the graph is encountered rather
 * than allowing it to continue until the distance to all nodes in the graph has been calculated
 */
template <typename FloatType>
class BGLEfficientOMPLSVSESolver : public BGLSolverBaseSVSE<FloatType>
{
public:
  using BGLSolverBaseSVSE<FloatType>::BGLSolverBaseSVSE;

  SearchResult<FloatType> search() override;
};

using BGLEfficientOMPLSVSESolverF = BGLEfficientOMPLSVSESolver<float>;
using BGLEfficientOMPLSVSESolverD = BGLEfficientOMPLSVSESolver<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_OMPL_OMPL_SOLVER_H
