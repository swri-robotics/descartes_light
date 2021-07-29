#pragma once

#include <descartes_light/test/utils.h>

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <numeric>
#include <random>
#include <vector>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
static std::mt19937 RAND_GEN(0);

template <typename FloatType>
typename State<FloatType>::Ptr generateRandomState(Eigen::Index dof)
{
  std::normal_distribution<FloatType> dist;
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> sol(dof);
  for (Eigen::Index i = 0; i < dof; ++i)
    sol(i) = dist(RAND_GEN);

  return std::make_shared<State<FloatType>>(sol);
}

template <typename FloatType>
RandomStateSampler<FloatType>::RandomStateSampler(const Eigen::Index dof,
                                                  const std::size_t n_samples,
                                                  const std::size_t zero_state_idx,
                                                  const FloatType state_cost)
  : dof_(dof), n_samples_(n_samples), zero_state_idx_(zero_state_idx), state_cost_(state_cost)
{
}

template <typename FloatType>
typename std::vector<StateSample<FloatType>> RandomStateSampler<FloatType>::sample() const
{
  // Generate some random joint states
  typename std::vector<StateSample<FloatType>> waypoints;
  waypoints.reserve(n_samples_);
  std::generate_n(std::back_inserter(waypoints), n_samples_, [this]() {
    return StateSample<FloatType>{ generateRandomState<FloatType>(dof_), state_cost_ };
  });

  // Set one of the joint states to all zeros
  waypoints.at(zero_state_idx_) = StateSample<FloatType>{
    std::make_shared<State<FloatType>>(Eigen::Matrix<FloatType, Eigen::Dynamic, 1>::Zero(this->dof_)), state_cost_
  };

  return waypoints;
}

template <typename FloatType>
NaiveEdgeEvaluator<FloatType>::NaiveEdgeEvaluator(const bool valid) : valid_(valid)
{
}

template <typename FloatType>
std::pair<bool, FloatType> NaiveEdgeEvaluator<FloatType>::evaluate(const State<FloatType>&,
                                                                   const State<FloatType>&) const
{
  return std::make_pair(valid_, 0.0);
}

template <typename FloatType>
NaiveStateEvaluator<FloatType>::NaiveStateEvaluator(const bool valid, const FloatType cost) : valid_(valid), cost_(cost)
{
}

template <typename FloatType>
std::pair<bool, FloatType> NaiveStateEvaluator<FloatType>::evaluate(const State<FloatType>&) const
{
  return std::make_pair(valid_, cost_);
}

template <class FloatType>
static std::vector<typename WaypointSampler<FloatType>::ConstPtr>
createSamplers(std::size_t dof, std::size_t n_waypoints, std::size_t samples_per_waypoint, FloatType state_cost)
{
  std::vector<typename WaypointSampler<FloatType>::ConstPtr> samplers;
  std::vector<std::size_t> zero_state_indices;
  samplers.reserve(n_waypoints);
  zero_state_indices.reserve(n_waypoints);

  std::uniform_int_distribution<std::size_t> dist(0, samples_per_waypoint - 1);

  // Create waypoint samplers
  for (std::size_t i = 0; i < n_waypoints; ++i)
  {
    auto zero_state_idx = dist(RAND_GEN);
    zero_state_indices.push_back(zero_state_idx);
    samplers.push_back(
        std::make_shared<RandomStateSampler<FloatType>>(dof, samples_per_waypoint, zero_state_idx, state_cost));
  }
  return samplers;
}

}  // namespace descartes_light
