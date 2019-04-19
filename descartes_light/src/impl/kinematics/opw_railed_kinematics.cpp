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

#include "descartes_light/impl/kinematics/opw_railed_kinematics.h"
#include <opw_kinematics/opw_utilities.h>
#include <iostream>
#include <math.h>

descartes_light::OPWRailedKinematics::OPWRailedKinematics(const opw_kinematics::Parameters<double> &params,
                                                          const Eigen::Isometry3d &world_to_rail_base,
                                                          const Eigen::Isometry3d &rail_base_to_robot_base,
                                                          const Eigen::Isometry3d &tool0_to_tip,
                                                          const Eigen::Matrix2d& rail_limits,
                                                          const Eigen::Vector2d& rail_sample_resolution,
                                                          const double robot_reach,
                                                          const IsValidFn& is_valid_fn,
                                                          const GetRedundentSolutionsFn& redundent_sol_fn)
  : params_(params)
  , world_to_rail_base_(world_to_rail_base)
  , rail_base_to_robot_base_(rail_base_to_robot_base)
  , tool0_to_tip_(tool0_to_tip)
  , rail_limits_(rail_limits)
  , rail_sample_resolution_(rail_sample_resolution)
  , robot_reach_(robot_reach)
  , is_valid_fn_(is_valid_fn)
  , redundent_sol_fn_(redundent_sol_fn)
{
}

bool descartes_light::OPWRailedKinematics::ik(const Eigen::Isometry3d &p, std::vector<double> &solution_set) const
{
  return ik(p, is_valid_fn_, redundent_sol_fn_, solution_set);
}

bool descartes_light::OPWRailedKinematics::ikAt(const Eigen::Isometry3d &p, const Eigen::Vector2d &rail_pose, std::vector<double> &solution_set) const
{
  return ikAt(p, rail_pose, is_valid_fn_, redundent_sol_fn_, solution_set);
}

bool descartes_light::OPWRailedKinematics::ik(const Eigen::Isometry3d& p,
                                              const IsValidFn& is_valid_fn,
                                              const GetRedundentSolutionsFn& redundent_sol_fn,
                                              std::vector<double>& solution_set) const
{
  // Tool pose in rail coordinate system
  Eigen::Isometry3d tool_pose = world_to_rail_base_.inverse() * p;

  const Eigen::Vector2d& rail_lower_limit = rail_limits_.col(0);
  const Eigen::Vector2d& rail_upper_limit = rail_limits_.col(1);

  const Eigen::Vector2d origin (tool_pose.translation().x() - rail_base_to_robot_base_.translation().x(), tool_pose.translation().y() - rail_base_to_robot_base_.translation().y());

  const double start_x = (origin.x() - robot_reach_ < rail_lower_limit.x()) ? rail_lower_limit.x() : origin.x() - robot_reach_;
  const double end_x = (origin.x() + robot_reach_ > rail_upper_limit.x()) ? rail_upper_limit.x() : origin.x() + robot_reach_;
  const double start_y = (origin.y() - robot_reach_ < rail_lower_limit.y()) ? rail_lower_limit.y() : origin.y() - robot_reach_;
  const double end_y = (origin.y() + robot_reach_ > rail_upper_limit.y()) ? rail_upper_limit.y() : origin.y() + robot_reach_;
  const double res_x = (end_x - start_x) / std::ceil((end_x - start_x) / rail_sample_resolution_.x());
  const double res_y = (end_y - start_y) / std::ceil((end_y - start_y) / rail_sample_resolution_.y());

  for (double x = start_x; x < end_x; x += res_x)
    for (double y = start_y; y < end_y; y += res_y)
      ikAt(p, Eigen::Vector2d(x, y), is_valid_fn, redundent_sol_fn, solution_set);

  return !solution_set.empty();
}

bool descartes_light::OPWRailedKinematics::ikAt(const Eigen::Isometry3d& p,
                                                const Eigen::Vector2d& rail_pose,
                                                const IsValidFn& is_valid_fn,
                                                const GetRedundentSolutionsFn& redundent_sol_fn,
                                                std::vector<double>& solution_set) const
{
  const Eigen::Isometry3d world_to_robot_base = world_to_rail_base_ * Eigen::Translation3d(rail_pose.x(), rail_pose.y(), 0.0) * rail_base_to_robot_base_;
  const Eigen::Isometry3d in_robot = world_to_robot_base.inverse() * p * tool0_to_tip_.inverse();

  std::array<double, 6*8> sols;
  opw_kinematics::inverse(params_, in_robot, sols.data());

  // Check the output
  for (int i = 0; i < 8; i++)
  {
    double* sol = sols.data() + 6 * i;
    if (opw_kinematics::isValid(sol))
    {
      opw_kinematics::harmonizeTowardZero(sol); // Modifies 'sol' in place

      std::vector<double> full_sol;
      full_sol.insert(end(full_sol), rail_pose.data(), rail_pose.data() + 2); // Insert the X-Y pose of the rail
      full_sol.insert(end(full_sol), sol, sol + 6); // And then insert the robot arm configuration

      if (is_valid_fn && redundent_sol_fn)
      {
        if (is_valid_fn(full_sol.data()))
          solution_set.insert(end(solution_set), full_sol.data(), full_sol.data() + 8);  // If good then add to solution set

        std::vector<double> redundent_sols = redundent_sol_fn(full_sol.data());
        if (!redundent_sols.empty())
        {
          int num_sol = redundent_sols.size()/8;
          for (int s = 0; s < num_sol; ++s)
          {
            double* redundent_sol = redundent_sols.data() + 8 * s;
            if (is_valid_fn(redundent_sol))
              solution_set.insert(end(solution_set), redundent_sol, redundent_sol + 8);  // If good then add to solution set
          }
        }
      }
      else if (is_valid_fn && !redundent_sol_fn)
      {
        if (is_valid_fn(full_sol.data()))
          solution_set.insert(end(solution_set), full_sol.data(), full_sol.data() + 8);  // If good then add to solution set
      }
      else if (!is_valid_fn && redundent_sol_fn)
      {

        solution_set.insert(end(solution_set), full_sol.data(), full_sol.data() + 8);  // If good then add to solution set

        std::vector<double> redundent_sols = redundent_sol_fn(full_sol.data());
        if (!redundent_sols.empty())
        {
          int num_sol = redundent_sols.size()/8;
          for (int s = 0; s < num_sol; ++s)
          {
            double* redundent_sol = redundent_sols.data() + 8 * s;
            solution_set.insert(end(solution_set), redundent_sol, redundent_sol + 8);  // If good then add to solution set
          }
        }
      }
      else
      {
        solution_set.insert(end(solution_set), full_sol.data(), full_sol.data() + 8);
      }
    }
  }

  return !solution_set.empty();
}

bool descartes_light::OPWRailedKinematics::fk(const double* pose, Eigen::Isometry3d& solution) const
{
  Eigen::Vector2d rail_pose;
  rail_pose(0) = pose[0];
  rail_pose(1) = pose[1];

  std::vector<double> opw_pose = {pose[2], pose[3], pose[4], pose[5], pose[6], pose[7]};

  return fkAt(rail_pose, opw_pose, solution);
}

bool descartes_light::OPWRailedKinematics::fkAt(const Eigen::Vector2d& rail_pose, const std::vector<double>& pose, Eigen::Isometry3d& solution) const
{
  solution = opw_kinematics::forward<double>(params_, pose.data());
  Eigen::Isometry3d rail_tf = Eigen::Isometry3d::Identity();
  rail_tf.translation().head(2) = rail_pose;
  solution = world_to_rail_base_ * rail_tf * rail_base_to_robot_base_ * solution * tool0_to_tip_;
}

void descartes_light::OPWRailedKinematics::analyzeIK(const Eigen::Isometry3d &p) const
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

