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
#include <boost/preprocessor.hpp>
DESCARTES_IGNORE_WARNINGS_POP

// Macro for explicitly instantiating a class with a variable number of template arguments
#define INSTANTIATE(T, ...) template class T<__VA_ARGS__>;

// Implementation macro for explicitly instantiating a class for a specific element of the Cartesian product (class_name, (template params))
#define INSTANTIATE_PRODUCT_IMPL(r, product) \
  INSTANTIATE(BOOST_PP_SEQ_HEAD(product), BOOST_PP_SEQ_ENUM(BOOST_PP_SEQ_TAIL(product)))

/**
 * Macro for instantiating a template class for a Cartesian product of float types and event visitor types
 * For example, instantiation of class Foo with types ((float)(double)) and visitors ((A)(B)) would result in 4 instantiations:
 *   - Foo<float, A>
 *   - Foo<float, B>
 *   - Foo<double, A>
 *   - Foo<double, B>
 */
#define INSTANTIATE_PRODUCT(TEMPLATE, FLOAT_TYPES, EVENT_VISITORS) \
  BOOST_PP_SEQ_FOR_EACH_PRODUCT(INSTANTIATE_PRODUCT_IMPL, ((TEMPLATE))(FLOAT_TYPES)(EVENT_VISITORS))

namespace descartes_light
{
// Event visitors
template struct early_terminator<boost::on_examine_vertex>;

template struct add_all_edges_dynamically<float, boost::on_examine_vertex>;
template struct add_all_edges_dynamically<double, boost::on_examine_vertex>;

// Explicit template instantiation
#define FLOAT_TYPES (double)(float)
#define DIJKSTRA_EVENT_VISITORS (boost::null_visitor)(early_terminator<boost::on_examine_vertex>)

// Partial implementations
INSTANTIATE_PRODUCT(BGLSolverBase, FLOAT_TYPES, DIJKSTRA_EVENT_VISITORS)
INSTANTIATE_PRODUCT(BGLSolverBaseSVSE, FLOAT_TYPES, DIJKSTRA_EVENT_VISITORS)
INSTANTIATE_PRODUCT(BGLSolverBaseSVDE, FLOAT_TYPES, DIJKSTRA_EVENT_VISITORS)

// BGL Dijkstra search
INSTANTIATE_PRODUCT(BGLDijkstraSVSESolver, FLOAT_TYPES, DIJKSTRA_EVENT_VISITORS)
INSTANTIATE_PRODUCT(BGLDijkstraSVDESolver, FLOAT_TYPES, DIJKSTRA_EVENT_VISITORS)

// Free functions
template SubGraph<double> createDecoratedSubGraph(const BGLGraph<double>& g);
template SubGraph<float> createDecoratedSubGraph(const BGLGraph<float>& g);

template void writeGraph(const std::string& filename, const BGLGraph<double>& graph);
template void writeGraph(const std::string& filename, const BGLGraph<float>& graph);

}  // namespace descartes_light
