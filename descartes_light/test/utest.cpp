#include <descartes_light/core/solver.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>
#include <descartes_light/solvers/bgl/bgl_dijkstra_solver.h>
#include <descartes_light/test/utils.h>
#include <descartes_light/test/solver_factory.h>

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
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

using Implementations = ::testing::Types<SolverFactory<LadderGraphSolverF>,
                                         SolverFactory<LadderGraphSolverD>,
                                         SolverFactory<BGLDijkstraSVSESolverF>,
                                         SolverFactory<BGLDijkstraSVSESolverD>,
                                         SolverFactory<BGLEfficientDijkstraSVSESolverF>,
                                         SolverFactory<BGLEfficientDijkstraSVSESolverD>,
                                         SolverFactory<BGLDijkstraSVDESolverF>,
                                         SolverFactory<BGLDijkstraSVDESolverD>,
                                         SolverFactory<BGLEfficientDijkstraSVDESolverF>,
                                         SolverFactory<BGLEfficientDijkstraSVDESolverD>>;

TYPED_TEST_CASE(SolverFixture, Implementations);

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
  ASSERT_DOUBLE_EQ(static_cast<double>(result.cost), static_cast<double>(total_cost));

  for (const auto& state : result.trajectory)
  {
    ASSERT_TRUE(state->values.isApprox(Eigen::Matrix<FloatType, Eigen::Dynamic, 1>::Zero(this->dof)));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
