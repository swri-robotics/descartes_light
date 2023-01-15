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
std::string LadderGraph<FloatType>::printString()
{
  std::stringstream ss;
  ss << "\nRung\t(Nodes)\t|# Outgoing Edges|\n";

  std::vector<std::size_t> failed_edges;
  for (std::size_t i = 0; i < rungs_.size(); i++)
  {
    Rung<FloatType> rung = rungs_[i];
    ss << i << "\t(" << rung.nodes.size() << ")\t|" << rung.numEdges() << "|\n";

    if (rung.numEdges() <= 0 && rung.nodes.size() > 0 && i < rungs_.size() - 1)
    {
      failed_edges.push_back(i);
    }
  }

  auto rungs_size = rungs_.size();
  if (!failed_edges.empty())
  {
    ss << "\nFailed edges: \n";
    for (auto failed_id : failed_edges)
    {
      Rung<FloatType> rung1 = rungs_[failed_id];
      if (rung1.nodes.empty())
        continue;
      Node<FloatType> node1 = rung1.nodes.front();
      typename State<FloatType>::ConstPtr state1 = node1.sample.state;

      Rung<FloatType> rung2 = rungs_[failed_id + 1];
      if (rung2.nodes.empty())
        continue;
      Node<FloatType> node2 = rung2.nodes.front();
      typename State<FloatType>::ConstPtr state2 = node2.sample.state;
      ss << "Rung # " << failed_id << "\n";
      for (Eigen::Index i = 0; i < state1->values.rows(); i++)
      {
        ss << std::setprecision(4) << std::fixed << "\t" << state1->values[i] << "\t|\t" << state2->values[i] << "\n";
      }
    }
  }
  ss << std::endl;
  return ss.str();
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_IMPL_LADDER_GRAPH_HPP
