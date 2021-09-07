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
#ifndef DESCARTES_LIGHT_SOLVERS_BGL_BGL_DFS_SOLVER_H
#define DESCARTES_LIGHT_SOLVERS_BGL_BGL_DFS_SOLVER_H

#include <descartes_light/solvers/bgl/bgl_solver.h>
#include <descartes_light/solvers/bgl/event_visitors.h>

namespace descartes_light
{
/**
 * @brief BGL solver implementation that constructs vertices and edges in the build function and uses a depth first
 * search with a specifiable visitor to search the graph
 */
template <typename FloatType, typename Visitors>
class BGLDepthFirstSVSESolver : public BGLSolverBaseSVSE<FloatType, Visitors>
{
public:
  using BGLSolverBaseSVSE<FloatType, Visitors>::BGLSolverBaseSVSE;
  SearchResult<FloatType> search() override;
};

using BGLDepthFirstSVSESolverF = BGLDepthFirstSVSESolver<float, early_terminator<boost::on_discover_vertex>>;
using BGLDepthFirstSVSESolverD = BGLDepthFirstSVSESolver<double, early_terminator<boost::on_discover_vertex>>;

/**
 * @brief BGL solver implementation that constructs vertices build function and uses a depth first search
 * with an edge-adding visitor to search the graph
 */
template <typename FloatType, typename Visitors>
class BGLDepthFirstSVDESolver : public BGLSolverBaseSVDE<FloatType, Visitors>
{
public:
  using BGLSolverBaseSVDE<FloatType, Visitors>::BGLSolverBaseSVDE;
  SearchResult<FloatType> search() override;
};

using BGLDepthFirstSVDESolverF = BGLDepthFirstSVDESolver<float, early_terminator<boost::on_discover_vertex>>;
using BGLDepthFirstSVDESolverD = BGLDepthFirstSVDESolver<double, early_terminator<boost::on_discover_vertex>>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_BGL_BGL_DFS_SOLVER_H
