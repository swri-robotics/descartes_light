#pragma once

#include <descartes_light/test/solver_factory.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>
#include <descartes_light/solvers/bgl/bgl_ladder_graph_solver.h>

namespace descartes_light
{
// Ladder graph solver Factory
template <typename FloatT>
struct SolverFactory<LadderGraphSolver<FloatT>>
{
  using FloatType = FloatT;
  typename Solver<FloatT>::Ptr create() const { return std::make_unique<LadderGraphSolver<FloatT>>(1); }
};

// Boost Ladder graph solver Factory
template <typename FloatT>
struct SolverFactory<BGLLadderGraphSolver<FloatT>>
{
  using FloatType = FloatT;
  typename Solver<FloatT>::Ptr create() const { return std::make_unique<BGLLadderGraphSolver<FloatT>>(1); }
};

}  // namespace descartes_light
