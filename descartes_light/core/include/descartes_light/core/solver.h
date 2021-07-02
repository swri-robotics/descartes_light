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

#include <descartes_light/core/waypoint_sampler.h>
#include <descartes_light/core/edge_evaluator.h>
#include <descartes_light/core/state_evaluator.h>
#include <descartes_light/types.h>

namespace descartes_light
{
struct BuildStatus
{
  /** @brief A vector of vertice indices in provided trajectory which failed */
  std::vector<std::size_t> failed_vertices;

  /** @brief A vector of edge indices in provided trajectory wich failed */
  std::vector<std::size_t> failed_edges;

  operator bool() const { return (failed_vertices.empty() && failed_edges.empty()); }
};

template <typename FloatType>
struct SearchResult
{
  /** @brief Lowest cost path through the graph */
  std::vector<typename State<FloatType>::ConstPtr> trajectory;
  /** @brief Path cost */
  FloatType cost;
};

template <typename FloatType>
class Solver
{
public:
  using FloatT = FloatType;
  using Ptr = std::shared_ptr<Solver<FloatType>>;
  using ConstPtr = std::shared_ptr<const Solver<FloatType>>;

  Solver() = default;
  virtual ~Solver() = default;
  Solver(const Solver&) = default;
  Solver& operator=(const Solver&) = default;
  Solver(Solver&&) noexcept = default;
  Solver& operator=(Solver&&) noexcept = default;

  /**
   * @brief Build the ladder graph
   * @param trajectory The waypoint samplers
   * @param edge_eval The edge evaluator; must either be size of one (used for each edge) or equal to one less than the
   * size of the trajectory (one per edge)
   * @param state_eval; The state evaluator; must either be size of one (used for each waypoint) or equal to the size of
   * the trajectory (one per waypoint)
   * @return BuildStatus is true if successfully built graph, otherwise false
   */
  BuildStatus build(const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
                    std::vector<typename EdgeEvaluator<FloatType>::ConstPtr> edge_evaluators,
                    std::vector<typename StateEvaluator<FloatType>::ConstPtr> state_evaluators)
  {
    if (edge_evaluators.size() == 1)
    {
      edge_evaluators.reserve(trajectory.size() - 1);
      std::fill_n(std::back_inserter(edge_evaluators), trajectory.size() - 2, edge_evaluators.front());
    }
    else if (edge_evaluators.size() != trajectory.size() - 1)
    {
      throw std::runtime_error("Invalid number of edge evaluators; size must equal 1 or trajectory size - 1");
    }

    if (state_evaluators.empty())
    {
      // State evaluators are not strictly necessary
    }
    else if (state_evaluators.size() == 1)
    {
      state_evaluators.reserve(trajectory.size());
      std::fill_n(std::back_inserter(state_evaluators), trajectory.size() - 1, state_evaluators.front());
    }
    else if (state_evaluators.size() != trajectory.size())
    {
      throw std::runtime_error("Invalid number of state evaluators; size must equal 1 or trajectory size");
    }

    return buildImpl(trajectory, edge_evaluators, state_evaluators);
  }

  /**
   * @brief Searches the graph
   * @throws This should throw an exception if it failed to find a solution.
   * @return The joint trajectory found
   */
  virtual SearchResult<FloatType> search() = 0;

protected:
  /**
   * @brief Implementation of the graph build function
   * @param trajectory Waypoint samplers
   * @param edge_eval Edge evaluators, size one less than the size of the trajectory
   * @param state_eval State evaluators, size equal to the size of the trajectory
   * @return
   */
  virtual BuildStatus buildImpl(const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
                                const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
                                const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_eval) = 0;
};

using SolverF = Solver<float>;
using SolverD = Solver<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_CORE_SOLVER_H
