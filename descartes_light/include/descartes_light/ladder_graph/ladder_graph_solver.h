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
#include <omp.h>
#include <vector>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/ladder_graph/ladder_graph.h>
#include <descartes_light/interface/solver.h>

namespace descartes_light
{
template <typename FloatType, template <typename, typename...> class ContainerType>
class LadderGraphSolver : public Solver<FloatType>
{
public:
  LadderGraphSolver(const std::size_t dof);

  bool build(const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
             const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
             const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_eval,
             int num_threads = omp_get_max_threads()) override;

  const std::vector<std::size_t>& getFailedVertices() const override { return failed_vertices_; }
  const std::vector<std::size_t>& getFailedEdges() const override { return failed_edges_; }

  std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> search() override;

private:
  LadderGraph<FloatType, ContainerType> graph_;
  std::vector<std::size_t> failed_vertices_;
  std::vector<std::size_t> failed_edges_;
};

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_DESCARTES_LIGHT_H