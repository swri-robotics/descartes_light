#include <descartes_light/core/solver.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>
#include <descartes_light/solvers/bgl/bgl_ladder_graph_solver.h>
#include <descartes_light/test/utils.h>
#include <descartes_light/test/solver_factory.h>

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <boost/format.hpp>
#include <chrono>
#include <gtest/gtest.h>
#include <functional>
#include <iostream>
#include <random>
#include <numeric>
#include <vector>
DESCARTES_IGNORE_WARNINGS_POP

using namespace descartes_light;

/**
 * @brief Test fixture for the solver interface
 */
template <typename SolverFactoryT>
class SolverFixture : public ::testing::Test
{
public:
  using FloatType = typename SolverFactoryT::FloatType;

  SolverFixture() : state_cost(static_cast<FloatType>(1.0))
  {
    samplers = createSamplers<FloatType>(static_cast<std::size_t>(dof), n_waypoints, samples_per_waypoint, state_cost);
  }

  const Eigen::Index dof{ 6 };
  const std::size_t n_waypoints{ 10 };
  const std::size_t samples_per_waypoint{ 4 };
  const FloatType state_cost;
  SolverFactoryT Factory;
  std::vector<typename WaypointSampler<FloatType>::ConstPtr> samplers;
};

template <typename SolverConfiguratorT>
class ParameterizedSolverFixture : public ::testing::Test
{
public:
  using FloatType = typename SolverConfiguratorT::FloatType;

  ParameterizedSolverFixture() : state_cost(static_cast<FloatType>(1.0)) {}

  const FloatType state_cost;
  SolverConfiguratorT configurator;
};

using Implementations = ::testing::Types<SolverFactory<LadderGraphSolverF>,
                                         SolverFactory<LadderGraphSolverD>,
                                         SolverFactory<BGLLadderGraphSolverF>,
                                         SolverFactory<BGLLadderGraphSolverD>>;

TYPED_TEST_CASE(SolverFixture, Implementations);
TYPED_TEST_CASE(ParameterizedSolverFixture, Implementations);

TYPED_TEST(SolverFixture, NoEdges)
{
  using FloatType = typename TypeParam::FloatType;
  typename Solver<FloatType>::Ptr solver = this->Factory.create();

  auto edge_eval = std::make_shared<const NaiveEdgeEvaluator<FloatType>>(false);
  auto state_eval = std::make_shared<const NaiveStateEvaluator<FloatType>>(true, this->state_cost);

  BuildStatus status = solver->build(this->samplers, { edge_eval }, { state_eval });
  ASSERT_FALSE(status);

  std::vector<std::size_t> expected_failed_edges(this->n_waypoints - 1);
  std::iota(expected_failed_edges.begin(), expected_failed_edges.end(), 0);

  ASSERT_TRUE(std::equal(status.failed_edges.begin(), status.failed_edges.end(), expected_failed_edges.begin()));
  ASSERT_EQ(status.failed_vertices.size(), 0);

  ASSERT_THROW(solver->search(), std::runtime_error);  // NOLINT
}

TYPED_TEST(SolverFixture, KnownPathTest)
{
  using FloatType = typename TypeParam::FloatType;
  typename Solver<FloatType>::Ptr solver = this->Factory.create();

  // Build a graph where one sample for each waypoint is an all zero state; evaluate edges using the Euclidean distance
  // metric Since each waypoint has an all-zero state, the shortest path should be through these samples
  auto edge_eval = std::make_shared<const EuclideanDistanceEdgeEvaluator<FloatType>>();
  auto state_eval = std::make_shared<const NaiveStateEvaluator<FloatType>>(true, this->state_cost);

  BuildStatus status = solver->build(this->samplers, { edge_eval }, { state_eval });
  ASSERT_TRUE(status);
  ASSERT_EQ(status.failed_vertices.size(), 0);
  ASSERT_EQ(status.failed_edges.size(), 0);

  SearchResult<FloatType> result = solver->search();
  ASSERT_EQ(result.trajectory.size(), this->n_waypoints);

  // Total path cost should be zero for edge costs and 2 * state_cost (one from sampling, one from state evaluation) for
  // state costs
  FloatType total_cost = static_cast<FloatType>(this->n_waypoints) * this->state_cost * 2;
  ASSERT_TRUE(std::abs(result.cost - total_cost) < std::numeric_limits<FloatType>::epsilon());

  for (const auto& state : result.trajectory)
  {
    ASSERT_TRUE(state->values.isApprox(Eigen::Matrix<FloatType, Eigen::Dynamic, 1>::Zero(this->dof)));
  }
}

TYPED_TEST(ParameterizedSolverFixture, ParameterizedKnownPath)
{
  using namespace std::chrono;

  using FloatType = typename TypeParam::FloatType;

  // Build a graph where one sample for each waypoint is an all zero state; evaluate edges using the Euclidean distance
  // metric Since each waypoint has an all-zero state, the shortest path should be through these samples
  auto edge_eval = std::make_shared<const EuclideanDistanceEdgeEvaluator<FloatType>>();
  auto state_eval = std::make_shared<const NaiveStateEvaluator<FloatType>>(true, this->state_cost);

  std::size_t dof_min = 4;
  std::size_t dof_max = 8;
  std::size_t n_waypoints_min = 12;
  std::size_t n_waypoints_max = 20;
  std::size_t samples_per_waypoint_min = 5;
  std::size_t samples_per_waypoint_max = 10;

  for (std::size_t dof = dof_min; dof <= dof_max; dof++)
  {
    for (std::size_t n_waypoints = n_waypoints_min; n_waypoints <= n_waypoints_max; n_waypoints++)
    {
      for (std::size_t samples_per_wp = samples_per_waypoint_min; samples_per_wp <= samples_per_waypoint_max;
           samples_per_wp++)
      {
        std::vector<typename WaypointSampler<FloatType>::ConstPtr> samplers =
            createSamplers<FloatType>(dof, n_waypoints, samples_per_wp, this->state_cost);

        typename Solver<FloatType>::Ptr solver = this->configurator.create();

        // building graph
        auto start_time = steady_clock::now();
        BuildStatus status = solver->build(samplers, { edge_eval }, { state_eval });
        double graph_build_time_elapsed =
            (static_cast<std::chrono::duration<double>>(steady_clock::now() - start_time)).count();

        ASSERT_TRUE(status);
        ASSERT_EQ(status.failed_vertices.size(), 0);
        ASSERT_EQ(status.failed_edges.size(), 0);

        // searching graph
        start_time = steady_clock::now();
        SearchResult<FloatType> result = solver->search();
        double graph_search_time_elapsed =
            (static_cast<std::chrono::duration<double>>(steady_clock::now() - start_time)).count();

        ASSERT_EQ(result.trajectory.size(), n_waypoints);

        // Total path cost should be zero for edge costs and 2 * state_cost (one from sampling, one from state
        // evaluation) for state costs
        FloatType total_cost = static_cast<FloatType>(n_waypoints) * this->state_cost * 2;
        ASSERT_TRUE(std::abs(result.cost - total_cost) < std::numeric_limits<FloatType>::epsilon());

        for (const auto& state : result.trajectory)
        {
          ASSERT_TRUE(state->values.isApprox(
              Eigen::Matrix<FloatType, Eigen::Dynamic, 1>::Zero(static_cast<Eigen::Index>(dof))));
        }

        // printing time results
        std::string inputs_str = boost::str(boost::format("Inputs: {dof: %f, n_waypoints: %f, samples: %f}") % dof %
                                            n_waypoints % samples_per_wp);
        std::cout << "======================= Graph Timing Results ======================= " << std::endl;
        std::cout << "\t" << inputs_str << std::endl;
        std::cout << "\tGraph Build Time: " << graph_build_time_elapsed << std::endl;
        std::cout << "\tGraph Search Time: " << graph_search_time_elapsed << std::endl;
      }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
