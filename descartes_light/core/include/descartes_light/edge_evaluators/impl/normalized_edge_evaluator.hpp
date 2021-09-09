#pragma once

#include <descartes_light/edge_evaluators/normalized_edge_evaluator.h>
#include <iomanip>

namespace descartes_light
{
template <typename FloatType>
NormalizedEdgeEvaluator<FloatType>::NormalizedEdgeEvaluator(typename EdgeEvaluator<FloatType>::ConstPtr evaluator,
                                                            FloatType min,
                                                            FloatType max)
  : evaluator_(std::move(evaluator)), min_(min), max_(max)
{
  if (std::abs(max_ - min_) < std::numeric_limits<FloatType>::epsilon())
    throw std::runtime_error("Limits cannot be the same");
}

template <typename FloatType>
std::pair<bool, FloatType> NormalizedEdgeEvaluator<FloatType>::evaluate(const State<FloatType>& start,
                                                                        const State<FloatType>& end) const
{
  std::pair<bool, FloatType> result = evaluator_->evaluate(start, end);

  // Handle the case when the cost is outside the bounds
  if (result.second < min_ || result.second > max_)
  {
    std::stringstream ss;
    ss << std::setprecision(4) << "Edge cost (" << result.second << ") does not lie within bounds [" << min_ << ", "
       << max_ << "]";
    throw std::runtime_error(ss.str());
  }

  result.second = (result.second - min_) / (max_ - min_);
  return result;
}

}  // namespace descartes_light
