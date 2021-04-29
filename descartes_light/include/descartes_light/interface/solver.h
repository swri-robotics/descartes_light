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
#ifndef DESCARTES_LIGHT_CORE_SOLVER_H
#define DESCARTES_LIGHT_CORE_SOLVER_H

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
#include <Eigen/Geometry>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/interface/waypoint_sampler.h>
#include <descartes_light/interface/edge_evaluator.h>
#include <descartes_light/interface/state_evaluator.h>

namespace descartes_light
{
template <typename FloatType>
class Solver
{
public:
  using Ptr = std::shared_ptr<Solver<FloatType>>;
  using ConstPtr = std::shared_ptr<const Solver<FloatType>>;

  virtual ~Solver() = default;

  /**
   * @brief Build the ladder graph
   * @param trajectory The waypoint samplers
   * @param edge_eval The edge evaluator.
   *        If empty it should be ignored.
   *        If size of one it should be used for each waypoint.
   * @param state_eval
   *        If empty it should be ignored.
   *        If size of one it should be used for each waypoint.
   * @param num_threads
   * @return True if successfully built graph, otherwise false
   */
  virtual bool build(const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
                     const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
                     const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_eval) = 0;

  /**
   * @brief Get a vector of vertice indices in provided trajectory which failed
   * @return The failed vertices
   */
  virtual const std::vector<std::size_t>& getFailedVertices() const = 0;

  /**
   * @brief Get a vector of edge indices in provided trajectory wich failed
   * @return The failed edges
   */
  virtual const std::vector<std::size_t>& getFailedEdges() const = 0;

  /**
   * @brief Search the graph
   * @throws This should throw an exception if it failed to find a solution.
   * @return The joint trajectory found
   */
  virtual std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> search() = 0;
};

using SolverF = Solver<float>;
using SolverD = Solver<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_CORE_SOLVER_H
