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
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/types.h>

namespace descartes_light
{
using ColorT = unsigned;

/**
 * @ Brief structure containing relevant information about a graph vertex
 */
template <typename FloatType>
struct Vertex
{
  /** @brief State sample information, assigned by user */
  StateSample<FloatType> sample;
  /** @brief Index of the "rung" of the ladder graph with which this node is associated */
  long rung_idx{ -1 };

  /** @brief Distance from the start node, assigned graph algorithm at search-time */
  FloatType distance{ std::numeric_limits<FloatType>::max() };
  /** @brief Search status "color" of the vertex: visited, opened, unvisited. Assigned by the graph algorithm at
   * search-time */
  ColorT color{ 0 };
};

using GraphvizAttributes = std::map<std::string, std::string>;

// clang-format off
template <typename FloatType>
using VertexProperty = boost::property<boost::vertex_index_t, std::size_t,
                                       boost::property<boost::vertex_attribute_t, GraphvizAttributes,
                                       Vertex<FloatType>>>;

template <typename FloatType>
using EdgeProperty = boost::property<boost::edge_weight_t, FloatType,
                     boost::property<boost::edge_index_t, std::size_t, // required for sub-graphs
                     boost::property<boost::edge_attribute_t, GraphvizAttributes>>>;

using GraphProperty = boost::property<boost::graph_name_t, std::string,
                      boost::property<boost::graph_graph_attribute_t, GraphvizAttributes,
                      boost::property<boost::graph_vertex_attribute_t, GraphvizAttributes,
                      boost::property<boost::graph_edge_attribute_t, GraphvizAttributes>>>>;

template <typename FloatType>
using BGLGraph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexProperty<FloatType>, EdgeProperty<FloatType>, GraphProperty>;
// clang-format on

template <typename FloatType>
using SubGraph = boost::subgraph<BGLGraph<FloatType>>;

template <typename FloatType>
using VertexDesc = typename BGLGraph<FloatType>::vertex_descriptor;

template <typename FloatType>
using VertexIt = typename BGLGraph<FloatType>::vertex_iterator;

template <typename FloatType>
using EdgeDesc = typename BGLGraph<FloatType>::edge_descriptor;

template <typename FloatType>
using EdgeIt = typename BGLGraph<FloatType>::edge_iterator;

using ColorTraits = boost::color_traits<ColorT>;

}  // namespace descartes_light
