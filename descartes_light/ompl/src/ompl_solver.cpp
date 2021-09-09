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
#include <descartes_light/ompl/impl/ompl_solver.hpp>
#include <descartes_light/ompl/impl/descartes_space.hpp>

namespace descartes_light
{
// Explicit template instantiation
// Full implementations
template class BGLOMPLSolver<double>;
template class BGLOMPLSolver<float>;

template class BGLOMPLRRTSolver<double>;
template class BGLOMPLRRTSolver<float>;

template class BGLOMPLRRTConnectSolver<double>;
template class BGLOMPLRRTConnectSolver<float>;

template class DescartesStateSampler<double>;
template class DescartesStateSampler<float>;

template class DescartesStateSpace<double>;
template class DescartesStateSpace<float>;

}  // namespace descartes_light
