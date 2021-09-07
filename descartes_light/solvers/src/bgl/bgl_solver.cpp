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
#include <descartes_light/solvers/bgl/impl/bgl_solver.hpp>
#include <descartes_light/solvers/bgl/impl/bgl_dijkstra_solver.hpp>
#include <descartes_light/solvers/bgl/impl/utils.hpp>

namespace descartes_light
{
// Explicit template instantiation
// Partial implementations
template class BGLSolverBase<double>;
template class BGLSolverBase<float>;

template class BGLSolverBaseSVDE<double>;
template class BGLSolverBaseSVDE<float>;

template class BGLSolverBaseSVSE<double>;
template class BGLSolverBaseSVSE<float>;

// Full Implementations
template class BGLDijkstraSVSESolver<double>;
template class BGLDijkstraSVSESolver<float>;

template class BGLEfficientDijkstraSVSESolver<double>;
template class BGLEfficientDijkstraSVSESolver<float>;

template class BGLDijkstraSVDESolver<double>;
template class BGLDijkstraSVDESolver<float>;

template class BGLEfficientDijkstraSVDESolver<double>;
template class BGLEfficientDijkstraSVDESolver<float>;
// Free functions
template SubGraph<double> createDecoratedSubGraph(const BGLGraph<double>& g);
template SubGraph<float> createDecoratedSubGraph(const BGLGraph<float>& g);

template void writeGraph(const std::string& filename, const BGLGraph<double>& graph);
template void writeGraph(const std::string& filename, const BGLGraph<float>& graph);

}  // namespace descartes_light
