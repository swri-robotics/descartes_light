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
#ifndef DESCARTES_LIGHT_EDGE_SAMPLER_H
#define DESCARTES_LIGHT_EDGE_SAMPLER_H

#include "descartes_light/core/edge_evaluator.h"

namespace descartes_light
{

class DistanceEdgeEvaluator : public EdgeEvaluator
{
public:
  DistanceEdgeEvaluator(const std::vector<double>& velocity_limits);

  bool evaluate(const Rung_<double>& from, const Rung_<double>& to,
                std::vector<LadderGraph<double>::EdgeList>& edges) override;

  std::vector<double> velocity_limits_;
};

}

#endif
