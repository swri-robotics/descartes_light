#pragma once

#include <descartes_light/test/solver_factory.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>
#include <descartes_light/solvers/bgl/bgl_dijkstra_solver.h>

namespace descartes_light
{
// Ladder graph solver factory
template <typename FloatType>
struct SolverFactory<LadderGraphSolver<FloatType>>
{
  typename Solver<FloatType>::Ptr create(long) const { return std::make_shared<LadderGraphSolver<FloatType>>(1); }
};

// Boost full Dijkstra graph solver factory
template <typename FloatType>
struct SolverFactory<BGLDijkstraSVSESolver<FloatType, boost::null_visitor>>
{
  using Visitors = boost::null_visitor;
  typename Solver<FloatType>::Ptr create(long) const
  {
    return std::make_shared<BGLDijkstraSVSESolver<FloatType, Visitors>>(Visitors(), 1);
  }
};

// Boost efficient Dijkstra graph solver factory
template <typename FloatType>
struct SolverFactory<BGLDijkstraSVSESolver<FloatType, early_terminator<boost::on_examine_vertex>>>
{
  using Visitors = early_terminator<boost::on_examine_vertex>;
  typename Solver<FloatType>::Ptr create(long n_waypoints) const
  {
    return std::make_shared<BGLDijkstraSVSESolver<FloatType, Visitors>>(Visitors(n_waypoints - 1), 1);
  }
};

// Dynamic Boost full Dijkstra graph solver factory
template <typename FloatType>
struct SolverFactory<BGLDijkstraSVDESolver<FloatType, boost::null_visitor>>
{
  using Visitors = boost::null_visitor;
  typename Solver<FloatType>::Ptr create(long) const
  {
    return std::make_shared<BGLDijkstraSVDESolver<FloatType, Visitors>>(Visitors(), 1);
  }
};

// Dynamic Boost efficient Dijkstra graph solver factory
template <typename FloatType>
struct SolverFactory<BGLDijkstraSVDESolver<FloatType, early_terminator<boost::on_examine_vertex>>>
{
  using Visitors = early_terminator<boost::on_examine_vertex>;
  typename Solver<FloatType>::Ptr create(long n_waypoints) const
  {
    return std::make_shared<BGLDijkstraSVDESolver<FloatType, Visitors>>(Visitors(n_waypoints - 1), 1);
  }
};

}  // namespace descartes_light
