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
#include <descartes_light/bgl/impl/bgl_solver.hpp>
#include <descartes_light/bgl/impl/bgl_dijkstra_solver.hpp>
#include <descartes_light/bgl/impl/bgl_dfs_solver.hpp>
#include <descartes_light/bgl/impl/utils.hpp>

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <boost/graph/visitors.hpp>
#include <boost/preprocessor.hpp>
DESCARTES_IGNORE_WARNINGS_POP

// Macro for explicitly instantiating a class with a variable number of template arguments
#define INSTANTIATE(T, ...) template class T<__VA_ARGS__>;

// Implementation macro for explicitly instantiating a class for a specific element of the Cartesian product
// (class_name, (template params))
#define INSTANTIATE_PRODUCT_IMPL(r, product)                                                                           \
  INSTANTIATE(BOOST_PP_SEQ_HEAD(product), BOOST_PP_SEQ_ENUM(BOOST_PP_SEQ_TAIL(product)))

/**
 * Macro for explicitly instantiating a template class for a Cartesian product of float types and event visitor types
 * For example, instantiation of class Foo with types ((float)(double)) and visitors ((A)(B)) would result in 4
 * instantiations:
 *   - Foo<float, A>
 *   - Foo<float, B>
 *   - Foo<double, A>
 *   - Foo<double, B>
 */
#define INSTANTIATE_PRODUCT(TEMPLATE, FLOAT_TYPES, EVENT_VISITORS)                                                     \
  BOOST_PP_SEQ_FOR_EACH_PRODUCT(INSTANTIATE_PRODUCT_IMPL, ((TEMPLATE))(FLOAT_TYPES)(EVENT_VISITORS))

// Implementation macro for explicitly instantiating a template class for a specific element of a sequence of types
#define INSTANTIATE_SEQ_IMPL(r, data, elem) INSTANTIATE(data, elem)

// Macro for explicitly instantiating a template class for a sequence of types
#define INSTANTIATE_SEQ(TEMPLATE, TYPES) BOOST_PP_SEQ_FOR_EACH(INSTANTIATE_SEQ_IMPL, TEMPLATE, TYPES)

namespace descartes_light
{
// Explicit template instantiation
// Event visitors
template struct early_terminator<boost::on_examine_vertex>;
template struct early_terminator<boost::on_discover_vertex>;

template struct add_all_edges_dynamically<float, boost::on_examine_vertex>;
template struct add_all_edges_dynamically<double, boost::on_examine_vertex>;

// Float type(s) with which to instantiate the solvers
#define FLOAT_TYPES (double)(float)
// Event visitor(s) used specifically by the Dijkstra solver
#define DIJKSTRA_EVENT_VISITORS (boost::null_visitor)(early_terminator<boost::on_examine_vertex>)
// Event visitor(s) used specifically by the DFS solver
#define DFS_EVENT_VISITORS (boost::null_visitor)(early_terminator<boost::on_discover_vertex>)

// Partial implementations
INSTANTIATE_SEQ(BGLSolverBase, FLOAT_TYPES)
INSTANTIATE_SEQ(BGLSolverBaseSVSE, FLOAT_TYPES)
INSTANTIATE_SEQ(BGLSolverBaseSVDE, FLOAT_TYPES)

// Full implementation
// BGL Dijkstra search
INSTANTIATE_PRODUCT(BGLDijkstraSVSESolver, FLOAT_TYPES, DIJKSTRA_EVENT_VISITORS)
INSTANTIATE_PRODUCT(BGLDijkstraSVDESolver, FLOAT_TYPES, DIJKSTRA_EVENT_VISITORS)

// BGL DFS
INSTANTIATE_PRODUCT(BGLDepthFirstSVSESolver, FLOAT_TYPES, DFS_EVENT_VISITORS)
INSTANTIATE_PRODUCT(BGLDepthFirstSVDESolver, FLOAT_TYPES, DFS_EVENT_VISITORS)

// Free functions
template SubGraph<double> createDecoratedSubGraph(const BGLGraph<double>& g);
template SubGraph<float> createDecoratedSubGraph(const BGLGraph<float>& g);

template void writeGraph(const std::string& filename, const BGLGraph<double>& graph);
template void writeGraph(const std::string& filename, const BGLGraph<float>& graph);

}  // namespace descartes_light
