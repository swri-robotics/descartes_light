/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef DESCARTES_SAMPLERS_EVALUATORS_TIMING_EDGE_EVALUATOR_H
#define DESCARTES_SAMPLERS_EVALUATORS_TIMING_EDGE_EVALUATOR_H

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/core/edge_evaluator.h>

namespace descartes_light
{
template <typename FloatType>
class TimingEdgeEvaluator : public EdgeEvaluator<FloatType>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // NOLINTNEXTLINE(modernize-pass-by-value)
  TimingEdgeEvaluator(const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& velocity_limits,
                      FloatType dt,
                      FloatType safety_factor);

  std::pair<bool, FloatType> evaluate(const State<FloatType>& start, const State<FloatType>& end) const override;

protected:
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> velocity_limits_;
  FloatType dt_;
  FloatType safety_factor_;
};

using TimingEdgeEvaluatorF = TimingEdgeEvaluator<float>;
using TimingEdgeEvaluatorD = TimingEdgeEvaluator<double>;

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_EVALUATORS_TIMING_EDGE_EVALUATOR_H
