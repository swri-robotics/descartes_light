/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2021, Southwest Research Institute
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
#ifndef DESCARTES_LIGHT_BGL_SOLVER_H
#define DESCARTES_LIGHT_BGL_SOLVER_H

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <thread>
#include <vector>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/core/solver.h>


// going to skip templating for the time being to get a build


//using listS because waypoints are currently added individually. Once preallocating, use VecS

namespace descartes_light
{

template <typename FloatType>
using  EdgeProperty = boost::property<boost::edge_weight_t, FloatType>;

//using listS because there is not currently any preallocation
template <typename FloatType>
using bglgraph = boost::adjacency_list<boost::listS, boost::listS, boost::directedS, StateSample<FloatType>, EdgeProperty<FloatType>>;

 // ToDo: Clean up this templating
template <typename FloatType>
using VertexDesc = typename bglgraph<FloatType>::vertex_descriptor;

template <typename FloatType>
class BGLLadderGraphSolver : public Solver<FloatType>
{
public:
  BGLLadderGraphSolver(const std::size_t dof, int num_threads = std::thread::hardware_concurrency());

  BuildStatus buildImpl(const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
                        const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
                        const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_eval) override;

  SearchResult<FloatType> search() override;

  std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs;
private:
  bglgraph<FloatType> graph_;
  int num_threads_;
};

using BGLLadderGraphSolverF = BGLLadderGraphSolver<float>;
using BGLLadderGraphSolverD = BGLLadderGraphSolver<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_BGL_SOLVER_H
