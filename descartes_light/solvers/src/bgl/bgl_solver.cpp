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

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <boost/graph/visitors.hpp>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
// Event visitors
template struct early_terminator<double, boost::on_examine_vertex>;
template struct early_terminator<float, boost::on_examine_vertex>;

template struct add_all_edges_dynamically<float, boost::on_examine_vertex>;
template struct add_all_edges_dynamically<double, boost::on_examine_vertex>;

// Explicit template instantiation
// Partial implementations
template class BGLSolverBase<double, early_terminator<double, boost::on_examine_vertex>>;
template class BGLSolverBase<float, early_terminator<float, boost::on_examine_vertex>>;
template class BGLSolverBase<double, boost::null_visitor>;
template class BGLSolverBase<float, boost::null_visitor>;

template class BGLSolverBaseSVDE<double, early_terminator<double, boost::on_examine_vertex>>;
template class BGLSolverBaseSVDE<float, early_terminator<float, boost::on_examine_vertex>>;
template class BGLSolverBaseSVDE<double, boost::null_visitor>;
template class BGLSolverBaseSVDE<float, boost::null_visitor>;

template class BGLSolverBaseSVSE<double, early_terminator<double, boost::on_examine_vertex>>;
template class BGLSolverBaseSVSE<float, early_terminator<float, boost::on_examine_vertex>>;
template class BGLSolverBaseSVSE<double, boost::null_visitor>;
template class BGLSolverBaseSVSE<float, boost::null_visitor>;

// Full Implementations
template class BGLDijkstraSVDESolver<double, early_terminator<double, boost::on_examine_vertex>>;
template class BGLDijkstraSVDESolver<float, early_terminator<float, boost::on_examine_vertex>>;
template class BGLDijkstraSVDESolver<double, boost::null_visitor>;
template class BGLDijkstraSVDESolver<float, boost::null_visitor>;

template class BGLDijkstraSVSESolver<double, early_terminator<double, boost::on_examine_vertex>>;
template class BGLDijkstraSVSESolver<float, early_terminator<float, boost::on_examine_vertex>>;
template class BGLDijkstraSVSESolver<double, boost::null_visitor>;
template class BGLDijkstraSVSESolver<float, boost::null_visitor>;

// Free functions
template SubGraph<double> createDecoratedSubGraph(const BGLGraph<double>& g);
template SubGraph<float> createDecoratedSubGraph(const BGLGraph<float>& g);

template void writeGraph(const std::string& filename, const BGLGraph<double>& graph);
template void writeGraph(const std::string& filename, const BGLGraph<float>& graph);

}  // namespace descartes_light
