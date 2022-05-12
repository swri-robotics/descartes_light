#pragma once

#include <descartes_light/test/solver_factory.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>
#include <descartes_light/bgl/bgl_dijkstra_solver.h>
#include <descartes_light/bgl/bgl_dfs_solver.h>

const static unsigned N_THREADS = 1;
#include <descartes_light/ompl/ompl_solver.h>
#include <descartes_light/ompl/descartes_space.h>

namespace descartes_light
{
// Ladder graph solver factory
template <typename FloatType>
struct SolverFactory<LadderGraphSolver<FloatType>>
{
  typename Solver<FloatType>::Ptr create(long) const
  {
    return std::make_shared<LadderGraphSolver<FloatType>>(N_THREADS);
  }
};

// BGL Dijkstra solver: static vertex, static edge, full search
template <typename FloatType>
struct SolverFactory<BGLDijkstraSVSESolver<FloatType, boost::null_visitor>>
{
  using Visitors = boost::null_visitor;
  typename Solver<FloatType>::Ptr create(long) const
  {
    return std::make_shared<BGLDijkstraSVSESolver<FloatType, Visitors>>(Visitors(), N_THREADS);
  }
};

// BGL Dijkstra solver: static vertex, static edge, terminate on encounter with last rung
template <typename FloatType>
struct SolverFactory<BGLDijkstraSVSESolver<FloatType, early_terminator<boost::on_examine_vertex>>>
{
  using Visitors = early_terminator<boost::on_examine_vertex>;
  typename Solver<FloatType>::Ptr create(long n_waypoints) const
  {
    return std::make_shared<BGLDijkstraSVSESolver<FloatType, Visitors>>(Visitors(n_waypoints - 1), N_THREADS);
  }
};

// BGL Dijkstra solver: static vertex, dynamic edge, full search
template <typename FloatType>
struct SolverFactory<BGLDijkstraSVDESolver<FloatType, boost::null_visitor>>
{
  using Visitors = boost::null_visitor;
  typename Solver<FloatType>::Ptr create(long) const
  {
    return std::make_shared<BGLDijkstraSVDESolver<FloatType, Visitors>>(Visitors(), N_THREADS);
  }
};

// BGL Dijkstra solver: static vertex, dynamic edge, terminate on encounter with last rung
template <typename FloatType>
struct SolverFactory<BGLDijkstraSVDESolver<FloatType, early_terminator<boost::on_examine_vertex>>>
{
  using Visitors = early_terminator<boost::on_examine_vertex>;
  typename Solver<FloatType>::Ptr create(long n_waypoints) const
  {
    return std::make_shared<BGLDijkstraSVDESolver<FloatType, Visitors>>(Visitors(n_waypoints - 1), N_THREADS);
  }
};

// BGL depth first solver: static vertex, static edge, full search
template <typename FloatType>
struct SolverFactory<BGLDepthFirstSVSESolver<FloatType, boost::null_visitor>>
{
  using Visitors = boost::null_visitor;
  typename Solver<FloatType>::Ptr create(long) const
  {
    return std::make_shared<BGLDepthFirstSVSESolver<FloatType, Visitors>>(Visitors(), N_THREADS);
  }
};

// BGL depth first solver: static vertex, static edge, terminate on encounter with last rung
template <typename FloatType>
struct SolverFactory<BGLDepthFirstSVSESolver<FloatType, early_terminator<boost::on_discover_vertex>>>
{
  using Visitors = early_terminator<boost::on_discover_vertex>;
  typename Solver<FloatType>::Ptr create(long n_waypoints) const
  {
    return std::make_shared<BGLDepthFirstSVSESolver<FloatType, Visitors>>(Visitors(n_waypoints - 1), N_THREADS);
  }
};

// BGL depth first solver: static vertex, dynamic edge, full search
template <typename FloatType>
struct SolverFactory<BGLDepthFirstSVDESolver<FloatType, boost::null_visitor>>
{
  using Visitors = boost::null_visitor;
  typename Solver<FloatType>::Ptr create(long) const
  {
    return std::make_shared<BGLDepthFirstSVDESolver<FloatType, Visitors>>(Visitors(), N_THREADS);
  }
};

// BGL depth first solver: static vertex, dynamic edge, terminate on encounter with last rung
template <typename FloatType>
struct SolverFactory<BGLDepthFirstSVDESolver<FloatType, early_terminator<boost::on_discover_vertex>>>
{
  using Visitors = early_terminator<boost::on_discover_vertex>;
  typename Solver<FloatType>::Ptr create(long n_waypoints) const
  {
    return std::make_shared<BGLDepthFirstSVDESolver<FloatType, Visitors>>(Visitors(n_waypoints - 1), N_THREADS);
  }
};

// Boost graph ompl RRT solver
template <typename FloatType>
struct SolverFactory<LadderGraphOMPLRRTSolver<FloatType>>
{
  typename Solver<FloatType>::Ptr create(long) const
  {
    // Setup RRT planner with a max cost of 2.1 from vertex to vertex (minimum value in tests is 2.0) and 10 seconds to
    // plan
    return std::make_shared<LadderGraphOMPLRRTSolver<FloatType>>(2.1, 10.0, 1);
  }
};

// Boost graph ompl RRT Connect solver
template <typename FloatType>
struct SolverFactory<LadderGraphOMPLRRTConnectSolver<FloatType>>
{
  typename Solver<FloatType>::Ptr create(long) const
  {
    // Setup RRT Connect planner with a max cost of 2.1 from vertex to vertex (minimum value in tests is 2.0) and 10
    // seconds to plan
    return std::make_shared<LadderGraphOMPLRRTConnectSolver<FloatType>>(2.1, 10.0, 1);
  }
};

}  // namespace descartes_light
