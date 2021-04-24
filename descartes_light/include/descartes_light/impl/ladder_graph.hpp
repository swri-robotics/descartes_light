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

#include <descartes_light/ladder_graph/ladder_graph.h>

namespace descartes_light
{
template <typename FloatType, template <typename, typename...> class ContainerType>
LadderGraph<FloatType, ContainerType>::LadderGraph(std::size_t dof) noexcept : dof_(dof)
{
  assert(dof != 0);
}

template <typename FloatType, template <typename, typename...> class ContainerType>
void LadderGraph<FloatType, ContainerType>::resize(std::size_t n_rungs)
{
  rungs_.resize(n_rungs);
}

template <typename FloatType, template <typename, typename...> class ContainerType>
std::size_t LadderGraph<FloatType, ContainerType>::size() const noexcept
{
  return rungs_.size();
}

template <typename FloatType, template <typename, typename...> class ContainerType>
std::size_t LadderGraph<FloatType, ContainerType>::dof() const noexcept
{
  return dof_;
}

template <typename FloatType, template <typename, typename...> class ContainerType>
ContainerType<Rung<FloatType, ContainerType>>& LadderGraph<FloatType, ContainerType>::getRungs() noexcept
{
  return rungs_;
}

template <typename FloatType, template <typename, typename...> class ContainerType>
const ContainerType<Rung<FloatType, ContainerType>>& LadderGraph<FloatType, ContainerType>::getRungs() const noexcept
{
  return rungs_;
}

template <typename FloatType, template <typename, typename...> class ContainerType>
Rung<FloatType, ContainerType>& LadderGraph<FloatType, ContainerType>::getRung(std::size_t rung_index) noexcept
{
  return const_cast<Rung<FloatType, ContainerType>&>(static_cast<const LadderGraph&>(*this).getRung(rung_index));
}

template <typename FloatType, template <typename, typename...> class ContainerType>
const Rung<FloatType, ContainerType>& LadderGraph<FloatType, ContainerType>::getRung(std::size_t rung_index) const
    noexcept
{
  assert(rung_index < rungs_.size());
  return (*std::next(rungs_.begin(), static_cast<long>(rung_index)));
}

template <typename FloatType, template <typename, typename...> class ContainerType>
std::size_t LadderGraph<FloatType, ContainerType>::rungSize(std::size_t rung_index) const noexcept
{
  return getRung(rung_index).nodes.size();
}

template <typename FloatType, template <typename, typename...> class ContainerType>
std::size_t LadderGraph<FloatType, ContainerType>::numVertices() const noexcept
{
  std::size_t count = 0;  // Add the size of each rung d
  for (const auto& rung : rungs_)
    count += rung.nodes.size();
  return count;
}

template <typename FloatType, template <typename, typename...> class ContainerType>
std::pair<std::size_t, bool> LadderGraph<FloatType, ContainerType>::indexOf(descartes_core::TrajectoryID id) const
    noexcept
{
  auto it = std::find_if(
      rungs_.cbegin(), rungs_.cend(), [id](const Rung<FloatType, ContainerType>& r) { return id == r.id; });
  if (it == rungs_.cend())
    return { 0u, false };
  else
    return { static_cast<std::size_t>(std::distance(rungs_.cbegin(), it)), true };
}

template <typename FloatType, template <typename, typename...> class ContainerType>
bool LadderGraph<FloatType, ContainerType>::isLast(std::size_t rung_index) const noexcept
{
  return rung_index + 1 == size();
}

template <typename FloatType, template <typename, typename...> class ContainerType>
bool LadderGraph<FloatType, ContainerType>::isFirst(std::size_t rung_index) const noexcept
{
  return rung_index == 0;
}

template <typename FloatType, template <typename, typename...> class ContainerType>
void LadderGraph<FloatType, ContainerType>::removeRung(std::size_t rung_index)
{
  rungs_.erase(std::next(rungs_.begin(), static_cast<long>(rung_index)));
}

template <typename FloatType, template <typename, typename...> class ContainerType>
void LadderGraph<FloatType, ContainerType>::clearNodes(std::size_t rung_index)
{
  std::next(rungs_.begin(), static_cast<long>(rung_index))->nodes.clear();
}

template <typename FloatType, template <typename, typename...> class ContainerType>
void LadderGraph<FloatType, ContainerType>::clearEdges(std::size_t rung_index)
{
  for (auto& n : std::next(rungs_.begin(), static_cast<long>(rung_index))->nodes)
    n.edges.clear();
}

template <typename FloatType, template <typename, typename...> class ContainerType>
void LadderGraph<FloatType, ContainerType>::insertRung(std::size_t rung_index)
{
  rungs_.insert(std::next(rungs_.begin(), static_cast<long>(rung_index)), Rung<FloatType, ContainerType>());
}

template <typename FloatType, template <typename, typename...> class ContainerType>
void LadderGraph<FloatType, ContainerType>::clear()
{
  rungs_.clear();
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_IMPL_LADDER_GRAPH_HPP
