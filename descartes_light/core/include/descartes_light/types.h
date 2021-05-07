#pragma once

#include <vector>
#include <Eigen/Geometry>

namespace descartes_light
{
/** @brief Definition of a state */
template <typename FloatType>
using State = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>;

template <typename FloatType>
struct StateSample
{
  StateSample(const State<FloatType>& state_, FloatType cost_) : state(state_), cost(cost_) {}
  StateSample(const State<FloatType>& state_) : StateSample(state_, static_cast<FloatType>(0.0)) {}

  /** @brief State values */
  State<FloatType> state;
  /** @brief State cost */
  FloatType cost;
};

}  // namespace descartes_light
