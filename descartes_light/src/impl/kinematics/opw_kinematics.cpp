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
#include "descartes_light/impl/kinematics/opw_kinematics.h"
#include <opw_kinematics/opw_utilities.h>
#include <iostream>

descartes_light::OPWKinematics::OPWKinematics(const opw_kinematics::Parameters<double>& params,
                                              const Eigen::Isometry3d& world_to_base,
                                              const Eigen::Isometry3d& tool0_to_tip)
  : params_(params)
  , world_to_base_(world_to_base)
  , tool0_to_tip_(tool0_to_tip)
{
}

bool descartes_light::OPWKinematics::ik(const Eigen::Isometry3d& p, std::vector<double>& solution_set) const
{
  Eigen::Isometry3d tool_pose = world_to_base_.inverse() * p * tool0_to_tip_.inverse();

  std::array<double, 6*8> sols;
  opw_kinematics::inverse(params_, tool_pose, sols.data());

  // Check the output
  for (int i = 0; i < 8; i++)
  {
    double* sol = sols.data() + 6 * i;
    if (opw_kinematics::isValid(sol))
    {
      opw_kinematics::harmonizeTowardZero(sol); // Modifies 'sol' in place

      // TODO: Joint limits?
      // If good then add to solution set
      solution_set.insert(end(solution_set), sol, sol + 6);
    }
  }

  return !solution_set.empty();
}

bool descartes_light::OPWKinematics::fk(const double* pose, Eigen::Isometry3d& solution) const
{
  solution = opw_kinematics::forward<double>(params_, pose);
  solution = world_to_base_ * solution * tool0_to_tip_.inverse();
  return true;
}
