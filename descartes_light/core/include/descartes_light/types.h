#pragma once

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <vector>
#include <Eigen/Geometry>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
/** @brief Definition of a state */
template <typename FloatType>
using State = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>;

template <typename FloatType>
using Array = Eigen::Array<FloatType, Eigen::Dynamic, 1>;

template <typename FloatType>
struct StateSample
{
  // NOLINTNEXTLINE(modernize-pass-by-value)
  StateSample(const State<FloatType>& state_, FloatType cost_) : state(state_), cost(cost_) {}
  // NOLINTNEXTLINE(modernize-pass-by-value)
  StateSample(const State<FloatType>& state_) : StateSample(state_, static_cast<FloatType>(0.0)) {}

  /** @brief State values */
  State<FloatType> state;
  /** @brief State cost */
  FloatType cost;
};

}  // namespace descartes_light
