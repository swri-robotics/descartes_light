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
#ifndef DESCARTES_LIGHT_IMPL_LADDER_GRAPH_HPP
#define DESCARTES_LIGHT_IMPL_LADDER_GRAPH_HPP

#include <descartes_light/solvers/ladder_graph/ladder_graph.h>
#include <iomanip>
#include <fstream>

namespace descartes_light
{
template <typename FloatType>
void LadderGraph<FloatType>::resize(std::size_t n_rungs)
{
  rungs_.resize(n_rungs);
}

template <typename FloatType>
std::size_t LadderGraph<FloatType>::size() const noexcept
{
  return rungs_.size();
}

template <typename FloatType>
std::vector<Rung<FloatType>>& LadderGraph<FloatType>::getRungs() noexcept
{
  return rungs_;
}

template <typename FloatType>
const std::vector<Rung<FloatType>>& LadderGraph<FloatType>::getRungs() const noexcept
{
  return rungs_;
}

template <typename FloatType>
Rung<FloatType>& LadderGraph<FloatType>::getRung(std::size_t rung_index) noexcept
{
  return const_cast<Rung<FloatType>&>(static_cast<const LadderGraph&>(*this).getRung(rung_index));
}

template <typename FloatType>
const Rung<FloatType>& LadderGraph<FloatType>::getRung(std::size_t rung_index) const noexcept
{
  assert(rung_index < rungs_.size());
  return (rungs_[rung_index]);
}

template <typename FloatType>
std::size_t LadderGraph<FloatType>::rungSize(std::size_t rung_index) const noexcept
{
  return getRung(rung_index).nodes.size();
}

template <typename FloatType>
std::size_t LadderGraph<FloatType>::numVertices() const noexcept
{
  std::size_t count = 0;  // Add the size of each rung d
  for (const auto& rung : rungs_)
    count += rung.nodes.size();
  return count;
}

template <typename FloatType>
std::pair<std::size_t, bool> LadderGraph<FloatType>::indexOf(descartes_core::TrajectoryID id) const noexcept
{
  auto it = std::find_if(rungs_.cbegin(), rungs_.cend(), [id](const Rung<FloatType>& r) { return id == r.id; });
  if (it == rungs_.cend())
    return { 0u, false };

  return { static_cast<std::size_t>(std::distance(rungs_.cbegin(), it)), true };
}

template <typename FloatType>
bool LadderGraph<FloatType>::isLast(std::size_t rung_index) const noexcept
{
  return rung_index + 1 == size();
}

template <typename FloatType>
bool LadderGraph<FloatType>::isFirst(std::size_t rung_index) const noexcept
{
  return rung_index == 0;
}

template <typename FloatType>
void LadderGraph<FloatType>::removeRung(std::size_t rung_index)
{
  rungs_.erase(std::next(rungs_.begin(), static_cast<long>(rung_index)));
}

template <typename FloatType>
void LadderGraph<FloatType>::clearNodes(std::size_t rung_index)
{
  rungs_[rung_index].nodes.clear();
}

template <typename FloatType>
void LadderGraph<FloatType>::clearEdges(std::size_t rung_index)
{
  for (auto& n : rungs_[rung_index].nodes)
    n.edges.clear();
}

template <typename FloatType>
void LadderGraph<FloatType>::insertRung(std::size_t rung_index)
{
  rungs_.insert(std::next(rungs_.begin(), static_cast<long>(rung_index)), Rung<FloatType>());
}

template <typename FloatType>
void LadderGraph<FloatType>::clear()
{
  rungs_.clear();
}

template <typename FloatType>
void LadderGraph<FloatType>::toDotGraph(std::string filepath,
                                        const std::vector<unsigned>& path) const
{
  std::ofstream out(filepath);
  if (!out)
    return;

  out << "digraph ladder {\n";
  out << "  rankdir=LR;\n";
  out << "  node [shape=box];\n";

  // Create subgraphs for each rung
  for (size_t rung_idx = 0; rung_idx < rungs_.size(); ++rung_idx)
  {
    out << "  subgraph cluster_" << rung_idx << " {\n";
    out << "    label=\"Rung " << rung_idx << "\";\n";
    out << "    style=dotted;\n";

    const auto& rung = rungs_[rung_idx];
    for (size_t node_idx = 0; node_idx < rung.nodes.size(); ++node_idx)
    {
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
    if (rung_idx < rungs_.size() - 1)
    {
      for (size_t node_idx = 0; node_idx < rung.nodes.size(); ++node_idx)
      {
        const auto& node = rung.nodes[node_idx];
        std::string from_node = "r" + std::to_string(rung_idx) + "n" + std::to_string(node_idx);

        for (const auto& edge : node.edges)
        {
          std::string to_node = "r" + std::to_string(rung_idx + 1) + "n" + std::to_string(edge.idx);
          // Fix: Properly format edge labels
          if (!path.empty() && rung_idx + 1 < path.size() && path[rung_idx] == node_idx &&
              path[rung_idx + 1] == edge.idx)
            out << "  " << from_node << " -> " << to_node << " [label=\"" << edge.cost
                << "\",color=green,penwidth=2.0];\n";
          else
            out << "  " << from_node << " -> " << to_node << " [label=\"" << edge.cost << "\"];\n";
        }
      }
    }
  }

  out << "}\n";
  out.close();
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_IMPL_LADDER_GRAPH_HPP
