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
#ifndef DESCARTES_LIGHT_DESCARTES_LIGHT_H
#define DESCARTES_LIGHT_DESCARTES_LIGHT_H

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <thread>
#include <vector>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/solvers/ladder_graph/ladder_graph.h>
#include <descartes_light/core/solver.h>

namespace descartes_light
{
template <typename FloatType>
class LadderGraphSolver : public Solver<FloatType>
{
public:
  LadderGraphSolver(std::size_t dof, int num_threads = static_cast<int>(std::thread::hardware_concurrency()));

  BuildStatus buildImpl(const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
                        const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
                        const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_eval) override;

  SearchResult<FloatType> search() override;

private:
  LadderGraph<FloatType> graph_;
  int num_threads_;
};

using LadderGraphSolverF = LadderGraphSolver<float>;
using LadderGraphSolverD = LadderGraphSolver<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_DESCARTES_LIGHT_H
