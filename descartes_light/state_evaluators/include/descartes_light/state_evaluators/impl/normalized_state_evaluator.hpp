#pragma once

#include <descartes_light/state_evaluators/normalized_state_evaluator.h>

namespace descartes_light
{
template <typename FloatType>
NormalizedStateEvaluator<FloatType>::NormalizedStateEvaluator(typename StateEvaluator<FloatType>::ConstPtr evaluator,
                                                              FloatType min,
                                                              FloatType max)
  : evaluator_(std::move(evaluator)), min_(min), max_(max)
{
  if (std::abs(max_ - min_) < std::numeric_limits<FloatType>::epsilon())
    throw std::runtime_error("Limits cannot be the same");
}

template <typename FloatType>
std::pair<bool, FloatType> NormalizedStateEvaluator<FloatType>::evaluate(const State<FloatType>& solution) const
{
  std::pair<bool, FloatType> result = evaluator_->evaluate(solution);
  result.second = (result.second - min_) / (max_ - min_);
  return result;
}

}  // namespace descartes_light
