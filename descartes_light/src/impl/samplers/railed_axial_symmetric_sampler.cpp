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
#include "descartes_light/impl/samplers/railed_axial_symmetric_sampler.h"
#include <iostream>

const static std::size_t dof = 8;

descartes_light::RailedAxialSymmetricSampler::RailedAxialSymmetricSampler(const Eigen::Isometry3d& tool_pose,
                                                                          const KinematicsInterfacePtr robot_kin,
                                                                          const double radial_sample_resolution,
                                                                          const CollisionInterfacePtr collision,
                                                                          const bool allow_collision)
  : tool_pose_(tool_pose)
  , kin_(robot_kin)
  , collision_(collision)
  , radial_sample_res_(radial_sample_resolution)
  , allow_collision_(allow_collision)
{}

bool descartes_light::RailedAxialSymmetricSampler::sample(std::vector<double>& solution_set)
{
  std::vector<double> buffer;

  const auto nSamplesInBuffer = [] (const std::vector<double>& v) -> std::size_t {
    return v.size() / dof;
  };

  double angle = -M_PI;

  while (angle <= M_PI) // loop over each waypoint
  {
    Eigen::Isometry3d p = tool_pose_ * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    kin_->ik(p, buffer);

    const auto n_sols = nSamplesInBuffer(buffer);
    for (std::size_t i = 0; i < n_sols; ++i)
    {
      const auto* sol_data = buffer.data() + i * dof;
      if (isCollisionFree(sol_data))
        solution_set.insert(end(solution_set), sol_data, sol_data + dof);
    }
    buffer.clear();

    angle += radial_sample_res_;
  } // redundancy resolution loop

  if (solution_set.empty() && allow_collision_)
    getBestSolution(solution_set);

  return !solution_set.empty();
}

bool descartes_light::RailedAxialSymmetricSampler::isCollisionFree(const double* vertex)
{
  return collision_->validate(vertex, dof);
}

bool descartes_light::RailedAxialSymmetricSampler::getBestSolution(std::vector<double>& solution_set)
{
  double distance = -std::numeric_limits<double>::max();
  std::vector<double> buffer;

  const auto nSamplesInBuffer = [] (const std::vector<double>& v) -> std::size_t {
    return v.size() / dof;
  };

  double angle = -M_PI;

  while (angle <= M_PI) // loop over each waypoint
  {
    Eigen::Isometry3d p = tool_pose_ * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    kin_->ik(p, buffer);

    const auto n_sols = nSamplesInBuffer(buffer);
    for (std::size_t i = 0; i < n_sols; ++i)
    {
      const auto* sol_data = buffer.data() + i * dof;
      double cur_distance = collision_->distance(sol_data, dof);
      if (cur_distance > distance)
      {
        distance = cur_distance;
        solution_set.clear();
        solution_set.insert(end(solution_set), sol_data, sol_data + dof);
      }
    }
    buffer.clear();

    angle += radial_sample_res_;
  } // redundancy resolution loop

  return !solution_set.empty();
}
