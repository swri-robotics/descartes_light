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
#include <descartes_ikfast/impl/ikfast_kinematics.hpp>
#include "fanuc_m20ia10l_manipulator_ikfast_solver.cpp"  // generated from ikfast for this robot
#include "descartes_ikfast_fanuc_m20ia10l_manipulator.h"

namespace descartes_ikfast_unit
{
// Explicit template instantiation
template class FanucM20ia10lKinematics<float>;
template class FanucM20ia10lKinematics<double>;

}  // namespace descartes_ikfast_unit
