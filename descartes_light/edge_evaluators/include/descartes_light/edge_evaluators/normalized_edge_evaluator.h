#pragma once

#include <descartes_light/core/edge_evaluator.h>

namespace descartes_light
{
/**
 * @brief Edge evaluator which normalizes the cost returned by an internal edge evaluator on [0.0, 1.0]
 */
template <typename FloatType>
class NormalizedEdgeEvaluator : public EdgeEvaluator<FloatType>
{
public:
  NormalizedEdgeEvaluator(typename EdgeEvaluator<FloatType>::ConstPtr evaluator, FloatType min, FloatType max);

  std::pair<bool, FloatType> evaluate(const State<FloatType>& start, const State<FloatType>& end) const override;

private:
  typename EdgeEvaluator<FloatType>::ConstPtr evaluator_;
  const FloatType min_;
  const FloatType max_;
};

using NormalizedEdgeEvaluatorD = NormalizedEdgeEvaluator<double>;
using NormalizedEdgeEvaluatorF = NormalizedEdgeEvaluator<float>;

}  // namespace descartes_light
