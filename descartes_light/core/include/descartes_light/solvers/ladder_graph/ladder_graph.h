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
#include <iomanip>
#include <fstream>
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

  std::size_t getEdgeCount()
  {
    std::size_t num_edges = 0;
    for (const auto& node : nodes)
      num_edges += node.edges.size();
    return num_edges;
  }
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

  void toDotGraph(std::string filepath, const std::vector<unsigned>& path = std::vector<unsigned>()) const
  {
    std::ofstream out(filepath);
    if (!out)
      return;

    out << "digraph ladder {\n";
    out << "  rankdir=LR;\n";
    out << "  node [shape=box];\n";

    // Create subgraphs for each rung
    for (size_t rung_idx = 0; rung_idx < rungs_.size(); ++rung_idx) {
      out << "  subgraph cluster_" << rung_idx << " {\n";
      out << "    label=\"Rung " << rung_idx << "\";\n";
      out << "    style=dotted;\n";
      
      const auto& rung = rungs_[rung_idx];
      for (size_t node_idx = 0; node_idx < rung.nodes.size(); ++node_idx) {
        std::string node_name = "r" + std::to_string(rung_idx) + "n" + std::to_string(node_idx);
        
        // Fix: Properly escape quotes and handle label closing
        out << "    " << node_name << " [label=\"" << node_idx;
        if (rung.nodes[node_idx].sample.cost >= 0)
          out << "\\nCost: " << rung.nodes[node_idx].sample.cost;

        out << "\\nState: ";
        if (rung.nodes[node_idx].sample.state)
          for (Eigen::Index i = 0; i < rung.nodes[node_idx].sample.state->values.size(); ++i)
            out << std::fixed << std::setprecision(3) << rung.nodes[node_idx].sample.state->values[i] << " ";
        
        if (!path.empty() && rung_idx < path.size() && path[rung_idx] == node_idx)
          out << "\",color=green,style=filled];\n";
        else
          out << "\"];\n";
      }
      out << "  }\n";

      // Create edges to next rung
      if (rung_idx < rungs_.size() - 1) {
        for (size_t node_idx = 0; node_idx < rung.nodes.size(); ++node_idx) {
          const auto& node = rung.nodes[node_idx];
          std::string from_node = "r" + std::to_string(rung_idx) + "n" + std::to_string(node_idx);
          
          for (const auto& edge : node.edges) {
            std::string to_node = "r" + std::to_string(rung_idx + 1) + "n" + std::to_string(edge.idx);
            // Fix: Properly format edge labels
            if (!path.empty() && rung_idx + 1 < path.size() && 
                path[rung_idx] == node_idx && path[rung_idx + 1] == edge.idx)
              out << "  " << from_node << " -> " << to_node 
                  << " [label=\"" << edge.cost << "\",color=green,penwidth=2.0];\n";
            else
              out << "  " << from_node << " -> " << to_node 
                  << " [label=\"" << edge.cost << "\"];\n";
          }
        }
      }
    }

    out << "}\n";
    out.close();
  }

  friend std::ostream& operator<<(std::ostream& out, const LadderGraph<FloatType>& ladder_graph)
  {
    out << "\nRung\t(Nodes)\t|# Outgoing Edges|\n";

    std::vector<std::size_t> failed_edges;
    for (std::size_t i = 0; i < ladder_graph.rungs_.size(); i++)
    {
      Rung<FloatType> rung = ladder_graph.rungs_[i];
      out << i << "\t(" << rung.nodes.size() << ")\t|" << rung.getEdgeCount() << "|\n";

      if (rung.getEdgeCount() <= 0 && rung.nodes.size() > 0 && i < ladder_graph.rungs_.size() - 1)
      {
        failed_edges.push_back(i);
      }
    }

    if (!failed_edges.empty())
    {
      out << "\nFailed edges: \n";
      for (auto failed_id : failed_edges)
      {
        Rung<FloatType> rung1 = ladder_graph.rungs_[failed_id];
        if (rung1.nodes.empty())
          continue;
        Node<FloatType> node1 = rung1.nodes.front();
        typename State<FloatType>::ConstPtr state1 = node1.sample.state;

        Rung<FloatType> rung2 = ladder_graph.rungs_[failed_id + 1];
        if (rung2.nodes.empty())
          continue;
        Node<FloatType> node2 = rung2.nodes.front();
        typename State<FloatType>::ConstPtr state2 = node2.sample.state;
        out << "Rung # " << failed_id << "\n";
        out << "\tNode 1\t|\tNode 2\t|\tDiff (Node2 - Node1)\n";
        out << "\t---------------------------------------\n";
        for (Eigen::Index i = 0; i < state1->values.rows(); i++)
        {
            out << std::setprecision(4) << std::fixed 
          << "\t" << state1->values[i] 
          << "\t|\t" << state2->values[i] 
          << "\t|\t" << state2->values[i] - state1->values[i] << "\n";
        }
        out << "\tTotal L2 Norm: " << (state2->values - state1->values).norm() << "\n";
      }
    }
    out << "\n";
    return out;
  }

private:
  std::vector<Rung<FloatType>> rungs_;
};

using LadderGraphF = LadderGraph<float>;
using LadderGraphD = LadderGraph<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_LADDER_GRAPH_H
