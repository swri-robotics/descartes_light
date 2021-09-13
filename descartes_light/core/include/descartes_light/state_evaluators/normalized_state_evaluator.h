#pragma once

#include <descartes_light/core/state_evaluator.h>

namespace descartes_light
{
template <typename FloatType>
class NormalizedStateEvaluator : public StateEvaluator<FloatType>
{
public:
  NormalizedStateEvaluator(typename StateEvaluator<FloatType>::ConstPtr evaluator, FloatType min, FloatType max);

  std::pair<bool, FloatType> evaluate(const State<FloatType>& solution) const override;

private:
  typename StateEvaluator<FloatType>::ConstPtr evaluator_;
  const FloatType min_;
  const FloatType max_;
};

using NormalizedStateEvaluatorF = NormalizedStateEvaluator<float>;
using NormalizedStateEvaluatorD = NormalizedStateEvaluator<double>;

}  // namespace descartes_light
