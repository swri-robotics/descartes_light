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
struct StateSample
{
  StateSample(State<FloatType> state_, FloatType cost_) : state(std::move(state_)), cost(cost_) {}
  StateSample(State<FloatType> state_) : StateSample(std::move(state_), static_cast<FloatType>(0.0)) {}

  /** @brief State values */
  State<FloatType> state;
  /** @brief State cost */
  FloatType cost;
};

}  // namespace descartes_light
