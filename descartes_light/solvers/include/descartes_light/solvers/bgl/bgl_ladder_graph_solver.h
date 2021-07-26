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
#pragma once

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <thread>
#include <vector>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/solvers/bgl/boost_graph_types.h>
#include <descartes_light/core/solver.h>

namespace descartes_light
{
/**
 * @brief Graph search leveraging the Boost Graph Library implementation of Dijkstra's algorithm
 */
template <typename FloatType>
class BGLLadderGraphSolver : public Solver<FloatType>
{
public:
  BGLLadderGraphSolver(unsigned num_threads = std::thread::hardware_concurrency());

  BuildStatus buildImpl(const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
                        const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
                        const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_eval) override;

  SearchResult<FloatType> search() override;

private:
  /**
   * @brief Helper function for reconstructing a path from a predecessor map
   * @param source
   * @param target
   * @param predecessor_map
   * @return
   */
  std::vector<typename State<FloatType>::ConstPtr>
  reconstructPath(const VertexDesc<FloatType>& source,
                  const VertexDesc<FloatType>& target,
                  const std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>& predecessor_map) const;

  unsigned num_threads_;
  /** @brief Graph representation of the planning problem */
  BGLGraph<FloatType> graph_;
  /** @brief Ladder graph representation of the graph vertices, used for creating edge connections */
  std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs_;
  /** @brief Artificial source vertex with a zero-cost edge to all vertices in the first ladder rung. All searches
   * should start from this vertex */
  VertexDesc<FloatType> source_;
};

using BGLLadderGraphSolverF = BGLLadderGraphSolver<float>;
using BGLLadderGraphSolverD = BGLLadderGraphSolver<double>;

}  // namespace descartes_light
