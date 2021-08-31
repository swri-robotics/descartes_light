#pragma once

#include <descartes_light/test/solver_factory.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>
#include <descartes_light/solvers/bgl/bgl_dijkstra_solver.h>
#include <descartes_light/solvers/bgl/dfs_add_all_solver.h>

namespace descartes_light
{
// Ladder graph solver factory
template <typename FloatType>
struct SolverFactory<LadderGraphSolver<FloatType>>
{
  typename Solver<FloatType>::Ptr create() const { return std::make_shared<LadderGraphSolver<FloatType>>(1); }
};

// Boost full Dijkstra graph solver factory
template <typename FloatType>
struct SolverFactory<BGLDijkstraSVSESolver<FloatType>>
{
  typename Solver<FloatType>::Ptr create() const { return std::make_shared<BGLDijkstraSVSESolver<FloatType>>(1); }
};

// Boost efficient Dijkstra graph solver factory
template <typename FloatType>
struct SolverFactory<BGLEfficientDijkstraSVSESolver<FloatType>>
{
  typename Solver<FloatType>::Ptr create() const
  {
    return std::make_shared<BGLEfficientDijkstraSVSESolver<FloatType>>(1);
  }
};

// Boost Add All Edges graph solver factory
template <typename FloatType>
struct SolverFactory<DFSAddAllSolver<FloatType>>
{
  typename Solver<FloatType>::Ptr create() const
  {
    return std::make_shared<DFSAddAllSolver<FloatType>>(1);
  }
};


}  // namespace descartes_light
