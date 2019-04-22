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
                                              const Eigen::Isometry3d& tool0_to_tip,
                                              const IsValidFn& is_valid_fn,
                                              const GetRedundentSolutionsFn& redundent_sol_fn)
  : params_(params)
  , world_to_base_(world_to_base)
  , tool0_to_tip_(tool0_to_tip)
  , is_valid_fn_(is_valid_fn)
  , redundent_sol_fn_(redundent_sol_fn)
{
}

bool descartes_light::OPWKinematics::ik(const Eigen::Isometry3d& p, std::vector<double>& solution_set) const
{
  return ik(p, is_valid_fn_, redundent_sol_fn_, solution_set);
}

bool descartes_light::OPWKinematics::ik(const Eigen::Isometry3d& p,
                                        const IsValidFn& is_valid_fn,
                                        const GetRedundentSolutionsFn& redundent_sol_fn,
                                        std::vector<double>& solution_set) const
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

      if (is_valid_fn && redundent_sol_fn)
      {
        if (is_valid_fn_(sol))
          solution_set.insert(end(solution_set), sol, sol + 6);  // If good then add to solution set

        std::vector<double> redundent_sols = redundent_sol_fn(sol);
        if (!redundent_sols.empty())
        {
          int num_sol = redundent_sols.size()/6;
          for (int s = 0; s < num_sol; ++s)
          {
            double* redundent_sol = redundent_sols.data() + 6 * s;
            if (is_valid_fn_(redundent_sol))
              solution_set.insert(end(solution_set), redundent_sol, redundent_sol + 6);  // If good then add to solution set
          }
        }
      }
      else if (is_valid_fn && !redundent_sol_fn)
      {
        if (is_valid_fn(sol))
          solution_set.insert(end(solution_set), sol, sol + 6);  // If good then add to solution set
      }
      else if (!is_valid_fn && redundent_sol_fn)
      {

        solution_set.insert(end(solution_set), sol, sol + 6);  // If good then add to solution set

        std::vector<double> redundent_sols = redundent_sol_fn(sol);
        if (!redundent_sols.empty())
        {
          int num_sol = redundent_sols.size()/6;
          for (int s = 0; s < num_sol; ++s)
          {
            double* redundent_sol = redundent_sols.data() + 6 * s;
            solution_set.insert(end(solution_set), redundent_sol, redundent_sol + 6);  // If good then add to solution set
          }
        }
      }
      else
      {
        solution_set.insert(end(solution_set), sol, sol + 6);
      }
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

void descartes_light::OPWKinematics::analyzeIK(const Eigen::Isometry3d &p) const
{
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "AnalyzeIK: ", ";");
  std::cout << p.matrix().format(CommaInitFmt) << std::endl;

  if (is_valid_fn_)
    std::cout << "\tIs Valid Function: true" << std::endl;
  else
    std::cout << "\tIs Valid Function: false" << std::endl;

  if (redundent_sol_fn_)
    std::cout << "\tGet Redundent Solutions Function: true" << std::endl;
  else
    std::cout << "\tGet Redundent Solutions Function: false" << std::endl;

  std::vector<double> solution_set;
  ik(p, nullptr, nullptr, solution_set);
  std::cout << "\tSampling without functions, found solutions: " << solution_set.size() / 8 << std::endl;

  solution_set.clear();
  ik(p, is_valid_fn_, nullptr, solution_set);
  std::cout << "\tSampling with only IsValid functions, found solutions: " << solution_set.size() / 8 << std::endl;

  solution_set.clear();
  ik(p, nullptr, redundent_sol_fn_, solution_set);
  std::cout << "\tSampling with only Redundent Solutions functions, found solutions: " << solution_set.size() / 8 << std::endl;

  solution_set.clear();
  ik(p, is_valid_fn_, redundent_sol_fn_, solution_set);
  std::cout << "\tSampling with both functions, found solutions: " << solution_set.size() / 8 << std::endl;
}
