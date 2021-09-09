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
#ifndef DESCARTES_LIGHT_LADDER_GRAPH_H
#define DESCARTES_LIGHT_LADDER_GRAPH_H

#include <descartes_light/types.h>
#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <cassert>
#include <vector>
#include <Eigen/Geometry>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_core
{
using TrajectoryID = std::size_t;
}  // namespace descartes_core

namespace descartes_light
{
template <typename FloatT>
struct Edge
{
  Edge() noexcept = default;
  Edge(FloatT cost, unsigned index) noexcept : cost{ cost }, idx{ index } {}

  /** @brief transition cost from vertex who owns this object to 'idx' in next rung */
  FloatT cost{ 0 };

  /** @brief from THIS rung to 'idx' into the NEXT rung */
  unsigned idx;
};

using EdgeF = Edge<float>;
using EdgeD = Edge<double>;

template <typename FloatT>
struct Node
{
  Node() noexcept = default;
  Node(const StateSample<FloatT>& sample_) noexcept : sample{ sample_ } {}  // NOLINT

  StateSample<FloatT> sample;

  /** @brief These are connects to other nodes */
  std::vector<Edge<FloatT>> edges;
};

using NodeF = Node<float>;
using NodeD = Node<double>;

template <typename FloatType>
struct Rung
{
  /** @brief corresponds to user's input ID */
  descartes_core::TrajectoryID id;

  /** @brief A vector of joint solutions */
  std::vector<Node<FloatType>> nodes;
};

using RungF = Rung<float>;
using RungD = Rung<double>;

/**
 * @brief LadderGraph is an adjacency list based, directed graph structure with vertices
 *        arranged into "rungs" which have connections only to vertices in the adjacent
 *        rungs. Assumes a fixed DOF.
 */
template <typename FloatType>
class LadderGraph
{
public:
  using EdgeList = std::vector<Edge<FloatType>>;
  using NodeList = std::vector<Node<FloatType>>;

  /**
   * @brief resize Resizes the internal ladder to have 'n_rung' rungs
   * @param n_rungs Number of individual rungs
   */
  void resize(std::size_t n_rungs);

  std::vector<Rung<FloatType>>& getRungs() noexcept;
  const std::vector<Rung<FloatType>>& getRungs() const noexcept;

  Rung<FloatType>& getRung(std::size_t rung_index) noexcept;
  const Rung<FloatType>& getRung(std::size_t rung_index) const noexcept;

  std::size_t rungSize(std::size_t rung_index) const noexcept;

  /** @brief numVertices Counts the total number of vertices in the graph */
  std::size_t numVertices() const noexcept;

  /**
   * @brief indexOf returns a pair describing whether the given ID is in the graph and if so, what
   *        index it has.
   * @param id The ID to
   * @return std::pair(index, was_found)
   */
  std::pair<std::size_t, bool> indexOf(descartes_core::TrajectoryID id) const noexcept;

  /**
   * @brief isLast tests to see if a given index is the last one in the graph
   */
  bool isLast(std::size_t rung_index) const noexcept;

  /**
   * @brief isFirst tests to see if given index is the first in the graph
   */
  bool isFirst(std::size_t rung_index) const noexcept;

  /**
   * @brief The number of rungs
   * @return
   */
  std::size_t size() const noexcept;

  void removeRung(std::size_t rung_index);

  void clearNodes(std::size_t rung_index);

  void clearEdges(std::size_t rung_index);

  /**
   * @brief insertRung Adds a new rung at the 'index'-th position. E.g., insertRung(0) will add a new
   *        rung to the beginning of the graph and the previous 0th index is now at 1.
   */
  void insertRung(std::size_t rung_index);

  /**
   * @brief Clears all existing rungs & associated data
   */
  void clear();

private:
  std::vector<Rung<FloatType>> rungs_;
};

using LadderGraphF = LadderGraph<float>;
using LadderGraphD = LadderGraph<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_LADDER_GRAPH_H
