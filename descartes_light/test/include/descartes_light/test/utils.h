#pragma once

#include <descartes_light/core/edge_evaluator.h>
#include <descartes_light/core/state_evaluator.h>
#include <descartes_light/core/waypoint_sampler.h>

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <random>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
/**
 * @brief Waypoint sampler that creates a set of random joint states with one all-zero joint state at a specified index
 */
template <typename FloatType>
class RandomStateSampler : public WaypointSampler<FloatType>
{
public:
  RandomStateSampler(const Eigen::Index dof,
                     const std::size_t n_samples,
                     const FloatType state_cost,
                     const std::size_t zero_state_idx,
                     std::shared_ptr<std::mt19937> rand_gen);

  typename std::vector<StateSample<FloatType>> sample() const override;

private:
  const Eigen::Index dof_;
  const std::size_t n_samples_;
  const FloatType state_cost_;
  const std::size_t zero_state_idx_;
  std::shared_ptr<std::mt19937> rand_gen_;
};

/**
 * @brief Edge evaluator class that evaluates all edges to be either valid or invalid, depending on the input, with a
 * cost of zero
 */
template <typename FloatType>
class NaiveEdgeEvaluator : public EdgeEvaluator<FloatType>
{
public:
  NaiveEdgeEvaluator(const bool valid);

  std::pair<bool, FloatType> evaluate(const State<FloatType>&, const State<FloatType>&) const override;

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
  NaiveStateEvaluator(const bool valid, const FloatType cost);

  std::pair<bool, FloatType> evaluate(const State<FloatType>&) const override;

private:
  const bool valid_;
  const FloatType cost_;
};

/**
 * @brief utility function to create a vector of RandomStateSampler
 * @param dof
 * @param n_waypoints
 * @param samples_per_waypoint
 * @return A vector
 */
template <class FloatType>
std::vector<typename WaypointSampler<FloatType>::ConstPtr>
createSamplers(std::size_t dof, std::size_t n_waypoints, std::size_t samples_per_waypoint, FloatType state_cost);

}  // namespace descartes_light
