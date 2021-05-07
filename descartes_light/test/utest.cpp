#include <descartes_light/core/solver.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>

#include <gtest/gtest.h>
#include <functional>
#include <iostream>
#include <random>
#include <numeric>
#include <vector>

using namespace descartes_light;

static std::mt19937 RAND_GEN(0);

template <typename FloatType>
State<FloatType> generateRandomState(Eigen::Index dof)
{
  std::normal_distribution<FloatType> dist;
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> sol(dof);
  for (Eigen::Index i = 0; i < dof; ++i)
    sol(i) = dist(RAND_GEN);

  return sol;
}

/**
 * @brief Waypoint sampler that creates a set of random joint states with one all-zero joint state at a specified index
 */
template <typename FloatType>
class RandomStateSampler : public WaypointSampler<FloatType>
{
public:
  RandomStateSampler(const Eigen::Index dof, const std::size_t n_samples, const std::size_t zero_state_idx)
    : dof_(dof), n_samples_(n_samples), zero_state_idx_(zero_state_idx)
  {
  }

  virtual std::vector<State<FloatType>> sample() const override
  {
    // Generate some random joint states
    std::vector<State<FloatType>> waypoints;
    waypoints.reserve(n_samples_);
    std::generate_n(
        std::back_inserter(waypoints), n_samples_, [this]() { return generateRandomState<FloatType>(dof_); });

    // Set one of the joint states to all zeros
    waypoints.at(zero_state_idx_) = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>::Zero(this->dof_);

    return waypoints;
  }

private:
  const Eigen::Index dof_;
  const std::size_t n_samples_;
  const std::size_t zero_state_idx_;
};

/**
 * @brief Edge evaluator class that evaluates all edges to be either valid or invalid, depending on the input, with a
 * cost of zero
 */
template <typename FloatType>
class NaiveEdgeEvaluator : public EdgeEvaluator<FloatType>
{
public:
  NaiveEdgeEvaluator(const bool valid) : valid_(valid) {}

  virtual std::pair<bool, FloatType> evaluate(const State<FloatType>&, const State<FloatType>&) const override
  {
    return std::make_pair(valid_, 0.0);
  }

private:
  const bool valid_;
};

/**
 * @brief State evaluator that evaluates all states to be either valid or invalid, depending on the input, with a cost
 * of zero
 */
template <typename FloatType>
class NaiveStateEvaluator : public StateEvaluator<FloatType>
{
public:
  NaiveStateEvaluator(const bool valid) : valid_(valid) {}

  virtual std::pair<bool, FloatType> evaluate(const State<FloatType>&) const override
  {
    return std::make_pair(valid_, 0.0);
  }

private:
  const bool valid_;
};

/**
 * @brief Object for configuring a Descartes solver for the unit test fixture
 */
template <typename SolverT>
struct SolverConfigurator
{
  using FloatType = typename SolverT::FloatT;
  typename Solver<FloatType>::Ptr create();
};

// Ladder graph solver configurator
template <typename FloatT>
struct SolverConfigurator<LadderGraphSolver<FloatT>>
{
  using FloatType = FloatT;
  typename Solver<FloatT>::Ptr create() { return std::make_unique<LadderGraphSolver<FloatT>>(6, 1); }
};
template struct SolverConfigurator<LadderGraphSolverF>;
template struct SolverConfigurator<LadderGraphSolverD>;

/**
 * @brief Test fixture for the solver interface
 */
template <typename SolverConfiguratorT>
class SolverFixture : public ::testing::Test
{
public:
  using FloatType = typename SolverConfiguratorT::FloatType;

  SolverFixture() : dof(6), n_waypoints(10), samples_per_waypoint(4)
  {
    samplers.reserve(n_waypoints);
    zero_state_indices_.reserve(n_waypoints);

    std::uniform_int_distribution<std::size_t> dist(0, samples_per_waypoint - 1);

    // Create waypoint samplers with ra
    for (std::size_t i = 0; i < n_waypoints; ++i)
    {
      auto zero_state_idx = dist(RAND_GEN);
      zero_state_indices_.push_back(zero_state_idx);
      samplers.push_back(std::make_shared<RandomStateSampler<FloatType>>(dof, samples_per_waypoint, dist(RAND_GEN)));
    }
  }

  SolverConfiguratorT configurator;
  const Eigen::Index dof;
  const std::size_t n_waypoints;
  const std::size_t samples_per_waypoint;
  std::vector<typename WaypointSampler<FloatType>::ConstPtr> samplers;
  std::vector<std::size_t> zero_state_indices_;
};

using Implementations =
    ::testing::Types<SolverConfigurator<LadderGraphSolverF>, SolverConfigurator<LadderGraphSolverD>>;

TYPED_TEST_CASE(SolverFixture, Implementations);

TYPED_TEST(SolverFixture, NoEdges)
{
  using FloatType = typename TypeParam::FloatType;
  typename Solver<FloatType>::Ptr solver = this->configurator.create();

  BuildStatus status = solver->build(this->samplers,
                                     { std::make_shared<const NaiveEdgeEvaluator<FloatType>>(false) },
                                     { std::make_shared<const NaiveStateEvaluator<FloatType>>(true) });
  ASSERT_FALSE(status);

  std::vector<std::size_t> expected_failed_edges(this->n_waypoints - 1);
  std::iota(expected_failed_edges.begin(), expected_failed_edges.end(), 0);

  ASSERT_TRUE(std::equal(status.failed_edges.begin(), status.failed_edges.end(), expected_failed_edges.begin()));
  ASSERT_EQ(status.failed_vertices.size(), 0);

  ASSERT_THROW(solver->search(), std::runtime_error);
}

TYPED_TEST(SolverFixture, KnownPathTest)
{
  using FloatType = typename TypeParam::FloatType;
  typename Solver<FloatType>::Ptr solver = this->configurator.create();

  // Build a graph where one sample for each waypoint is an all zero state; evaluate edges using the Euclidean distance
  // metric Since each waypoint has an all-zero state, the shortest path should be through these samples with a total
  // cost of zero
  BuildStatus status = solver->build(this->samplers,
                                     { std::make_shared<const EuclideanDistanceEdgeEvaluator<FloatType>>() },
                                     { std::make_shared<const NaiveStateEvaluator<FloatType>>(true) });
  ASSERT_TRUE(status);
  ASSERT_EQ(status.failed_vertices.size(), 0);
  ASSERT_EQ(status.failed_edges.size(), 0);

  SearchResult<FloatType> result = solver->search();
  ASSERT_EQ(result.trajectory.size(), this->n_waypoints);
  ASSERT_TRUE(std::abs(result.cost) < std::numeric_limits<FloatType>::epsilon());

  for (const auto& state : result.trajectory)
  {
    ASSERT_TRUE(state.isApprox(Eigen::Matrix<FloatType, Eigen::Dynamic, 1>::Zero(this->dof)));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
