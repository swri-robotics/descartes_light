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
#include "descartes_light/samplers/cartesian_point_sampler.h"
#include <iostream>

const static std::size_t opw_dof = 6;

descartes_light::CartesianPointSampler::CartesianPointSampler(const Eigen::Isometry3d& tool_pose,
                                                              const KinematicsInterfacePtr robot_kin,
                                                              const CollisionInterfacePtr collision)
  : tool_pose_(tool_pose)
  , kin_(robot_kin)
  , collision_(std::move(collision))
{
}

bool descartes_light::CartesianPointSampler::sample(std::vector<double>& solution_set)
{
  std::vector<double> buffer;
  kin_->ik(tool_pose_, buffer);

  const auto nSamplesInBuffer = [] (const std::vector<double>& v) -> std::size_t {
    return v.size() / opw_dof;
  };

  const auto n_sols = nSamplesInBuffer(buffer);

  for (std::size_t i = 0; i < n_sols; ++i)
  {
    const auto* sol_data = buffer.data() + i * opw_dof;
    if (isCollisionFree(sol_data))
      solution_set.insert(end(solution_set), sol_data, sol_data + opw_dof);
  }

  return !solution_set.empty();
}

bool descartes_light::CartesianPointSampler::isCollisionFree(const double* vertex)
{
  return collision_->validate(vertex, opw_dof);
}
