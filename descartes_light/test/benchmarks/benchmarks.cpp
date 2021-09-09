#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_light/test/utils.h>
#include <descartes_light/test/solver_factory.h>
// Solvers
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>
#include <descartes_light/bgl/bgl_dijkstra_solver.h>
#include <descartes_light/bgl/bgl_dfs_solver.h>

DESCARTES_IGNORE_WARNINGS_PUSH
#include <boost/core/demangle.hpp>
#include <boost/format.hpp>
#include <chrono>
#include <iostream>
DESCARTES_IGNORE_WARNINGS_POP

using namespace descartes_light;

/**
 * @brief Benchmark for testing general solver performance on a variety of planning problem sizes
 */
template <typename SolverT>
void benchmark(const SolverFactory<SolverT>& factory)
{
  std::cout << "\n==============================================" << std::endl;
  std::cout << boost::core::demangle(typeid(SolverT).name()) << std::endl;
  std::cout << "==============================================\n" << std::endl;

  using FloatType = typename SolverT::FloatT;
  using Clock = std::chrono::steady_clock;

  // Parameterize the size of the planning problem in terms of the number of degrees of freedom, number of waypoints in
  // the problem, and number of samples per waypoint
  const std::vector<std::size_t> v_dof{ 6, 8, 10 };
  const std::vector<std::size_t> v_waypoints{ 10, 50, 100 };
  const std::vector<std::size_t> v_samples{ 10, 50, 100 };

  const FloatType state_cost = static_cast<FloatType>(1.0);

  // Build a graph where one sample for each waypoint is an all zero state; evaluate edges using the Euclidean distance
  // metric Since each waypoint has an all-zero state, the shortest path should be through these samples
  auto edge_eval = std::make_shared<const EuclideanDistanceEdgeEvaluator<FloatType>>();
  auto state_eval = std::make_shared<const NaiveStateEvaluator<FloatType>>(true, state_cost);

  for (std::size_t dof : v_dof)
  {
    for (std::size_t n_waypoints : v_waypoints)
    {
      for (std::size_t samples_per_wp : v_samples)
      {
        // Create the waypoint samplers
        std::vector<typename WaypointSampler<FloatType>::ConstPtr> samplers =
            createSamplers<FloatType>(dof, n_waypoints, samples_per_wp, state_cost);

        // Create the solver
        typename descartes_light::Solver<FloatType>::Ptr solver = factory.create(static_cast<long>(n_waypoints));

        // Build the graph
        auto start_time = Clock::now();
        BuildStatus status = solver->build(samplers, { edge_eval }, { state_eval });
        double graph_build_time = std::chrono::duration<double>(Clock::now() - start_time).count();

        // Search the graph
        start_time = Clock::now();
        SearchResult<FloatType> result = solver->search();
        double graph_search_time = std::chrono::duration<double>(Clock::now() - start_time).count();

        // printing time results
        std::string params = boost::str(boost::format("Inputs: {dof: %lu, n_waypoints: %lu, samples: %lu}") % dof %
                                        n_waypoints % samples_per_wp);
        std::cout << params << std::endl;
        std::cout << "\tGraph Build Time (s): " << graph_build_time << std::endl;
        std::cout << "\tGraph Search Time (s): " << graph_search_time << std::endl;
      }
    }
  }
}

int main(int, char**)
{
  // Ladder graph
  benchmark(SolverFactory<LadderGraphSolverD>());
  benchmark(SolverFactory<LadderGraphSolverF>());

  // BGL Dijkstra full search, static vertex static edge
  benchmark(SolverFactory<BGLDijkstraSVSESolver<double, boost::null_visitor>>());
  benchmark(SolverFactory<BGLDijkstraSVSESolver<float, boost::null_visitor>>());

  // BGL Dijkstra early termination, static vertex static edge
  benchmark(SolverFactory<BGLDijkstraSVSESolverD>());
  benchmark(SolverFactory<BGLDijkstraSVSESolverF>());

  // BGL Dijkstra full search, static vertex dynamic edge
  benchmark(SolverFactory<BGLDijkstraSVDESolver<double, boost::null_visitor>>());
  benchmark(SolverFactory<BGLDijkstraSVDESolver<float, boost::null_visitor>>());

  // BGL Dijkstra early termination, static vertex, dynamic edge
  benchmark(SolverFactory<BGLDijkstraSVDESolverD>());
  benchmark(SolverFactory<BGLDijkstraSVDESolverF>());

  // BGL depth first full search, static vertex static edge
  benchmark(SolverFactory<BGLDepthFirstSVSESolver<double, boost::null_visitor>>());
  benchmark(SolverFactory<BGLDepthFirstSVSESolver<float, boost::null_visitor>>());

  // BGL depth first early termination, static vertex static edge
  benchmark(SolverFactory<BGLDepthFirstSVSESolverD>());
  benchmark(SolverFactory<BGLDepthFirstSVSESolverF>());

  // BGL depth first full search, static vertex dynamic edge
  benchmark(SolverFactory<BGLDepthFirstSVDESolver<double, boost::null_visitor>>());
  benchmark(SolverFactory<BGLDepthFirstSVDESolver<float, boost::null_visitor>>());

  // BGL depth first early termination, static vertex, dynamic edge
  benchmark(SolverFactory<BGLDepthFirstSVDESolverD>());
  benchmark(SolverFactory<BGLDepthFirstSVDESolverF>());

  return 0;
}
