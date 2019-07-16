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
#ifndef DESCARTES_OPW_IMPL_RAILED_KINEMATICS_OPW_HPP
#define DESCARTES_OPW_IMPL_RAILED_KINEMATICS_OPW_HPP

#include "descartes_opw/descartes_opw_railed_kinematics.h"
#include <opw_kinematics/opw_utilities.h>
#include <console_bridge/console.h>

namespace descartes_light
{

template<typename FloatType>
OPWRailedKinematics<FloatType>::OPWRailedKinematics(const opw_kinematics::Parameters<FloatType> &params,
                                                    const Eigen::Transform<FloatType, 3, Eigen::Isometry> &world_to_rail_base,
                                                    const Eigen::Transform<FloatType, 3, Eigen::Isometry> &rail_base_to_robot_base,
                                                    const Eigen::Transform<FloatType, 3, Eigen::Isometry> &tool0_to_tip,
                                                    const Eigen::Matrix<FloatType, 2, 2>& rail_limits,
                                                    const Eigen::Matrix<FloatType, 2, 1>& rail_sample_resolution,
                                                    const FloatType robot_reach,
                                                    const IsValidFn<FloatType>& is_valid_fn,
                                                    const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn)
  : params_(params)
  , world_to_rail_base_(world_to_rail_base)
  , rail_base_to_robot_base_(rail_base_to_robot_base)
  , tool0_to_tip_(tool0_to_tip)
  , rail_limits_(rail_limits)
  , rail_sample_resolution_(rail_sample_resolution)
  , robot_reach_(robot_reach)
  , is_valid_fn_(is_valid_fn)
  , redundant_sol_fn_(redundant_sol_fn)
{
}

template<typename FloatType>
bool OPWRailedKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry> &p,
                                        std::vector<FloatType> &solution_set) const
{
  return OPWRailedKinematics<FloatType>::ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
}

template<typename FloatType>
bool OPWRailedKinematics<FloatType>::ikAt(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                          const Eigen::Matrix<FloatType, 2, 1>& rail_pose,
                                          std::vector<FloatType>& solution_set) const
{
  return OPWRailedKinematics<FloatType>::ikAt(p, rail_pose, is_valid_fn_, redundant_sol_fn_, solution_set);
}

template<typename FloatType>
bool OPWRailedKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                        const IsValidFn<FloatType>& is_valid_fn,
                                        const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
                                        std::vector<FloatType>& solution_set) const
{
  // Tool pose in rail coordinate system
  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool_pose = world_to_rail_base_.inverse() * p;

  const Eigen::Matrix<FloatType, 2, 1>& rail_lower_limit = rail_limits_.col(0);
  const Eigen::Matrix<FloatType, 2, 1>& rail_upper_limit = rail_limits_.col(1);

  const Eigen::Matrix<FloatType, 2, 1> origin (tool_pose.translation().x() - rail_base_to_robot_base_.translation().x(), tool_pose.translation().y() - rail_base_to_robot_base_.translation().y());

  const FloatType start_x = (origin.x() - robot_reach_ < rail_lower_limit.x()) ? rail_lower_limit.x() : origin.x() - robot_reach_;
  const FloatType end_x = (origin.x() + robot_reach_ > rail_upper_limit.x()) ? rail_upper_limit.x() : origin.x() + robot_reach_;
  const FloatType start_y = (origin.y() - robot_reach_ < rail_lower_limit.y()) ? rail_lower_limit.y() : origin.y() - robot_reach_;
  const FloatType end_y = (origin.y() + robot_reach_ > rail_upper_limit.y()) ? rail_upper_limit.y() : origin.y() + robot_reach_;
  const FloatType res_x = (end_x - start_x) / std::ceil((end_x - start_x) / rail_sample_resolution_.x());
  const FloatType res_y = (end_y - start_y) / std::ceil((end_y - start_y) / rail_sample_resolution_.y());

  for (FloatType x = start_x; x < end_x; x += res_x)
    for (FloatType y = start_y; y < end_y; y += res_y)
      ikAt(p, Eigen::Matrix<FloatType, 2, 1>(x, y), is_valid_fn, redundant_sol_fn, solution_set);

  return !solution_set.empty();
}

template<typename FloatType>
bool OPWRailedKinematics<FloatType>::ikAt(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                          const Eigen::Matrix<FloatType, 2, 1>& rail_pose,
                                          const IsValidFn<FloatType>& is_valid_fn,
                                          const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
                                          std::vector<FloatType>& solution_set) const
{
  const Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_robot_base = world_to_rail_base_ * Eigen::Translation<FloatType, 3>(rail_pose.x(), rail_pose.y(), static_cast<FloatType>(0.0)) * rail_base_to_robot_base_;
  const Eigen::Transform<FloatType, 3, Eigen::Isometry> in_robot = world_to_robot_base.inverse() * p * tool0_to_tip_.inverse();

  std::array<FloatType, 6*8> sols;
  opw_kinematics::inverse(params_, in_robot, sols.data());

  // Check the output
  for (int i = 0; i < 8; i++)
  {
    FloatType* sol = sols.data() + 6 * i;
    if (opw_kinematics::isValid(sol))
    {
      opw_kinematics::harmonizeTowardZero(sol); // Modifies 'sol' in place

      std::vector<FloatType> full_sol;
      full_sol.insert(end(full_sol), rail_pose.data(), rail_pose.data() + 2); // Insert the X-Y pose of the rail
      full_sol.insert(end(full_sol), sol, sol + 6); // And then insert the robot arm configuration

      if (is_valid_fn && redundant_sol_fn)
      {
        if (is_valid_fn(full_sol.data()))
          solution_set.insert(end(solution_set), full_sol.data(), full_sol.data() + 8);  // If good then add to solution set

        std::vector<FloatType> redundant_sols = redundant_sol_fn(full_sol.data());
        if (!redundant_sols.empty())
        {
          int num_sol = redundant_sols.size()/8;
          for (int s = 0; s < num_sol; ++s)
          {
            FloatType* redundant_sol = redundant_sols.data() + 8 * s;
            if (is_valid_fn(redundant_sol))
              solution_set.insert(end(solution_set), redundant_sol, redundant_sol + 8);  // If good then add to solution set
          }
        }
      }
      else if (is_valid_fn && !redundant_sol_fn)
      {
        if (is_valid_fn(full_sol.data()))
          solution_set.insert(end(solution_set), full_sol.data(), full_sol.data() + 8);  // If good then add to solution set
      }
      else if (!is_valid_fn && redundant_sol_fn)
      {

        solution_set.insert(end(solution_set), full_sol.data(), full_sol.data() + 8);  // If good then add to solution set

        std::vector<FloatType> redundant_sols = redundant_sol_fn(full_sol.data());
        if (!redundant_sols.empty())
        {
          int num_sol = redundant_sols.size()/8;
          for (int s = 0; s < num_sol; ++s)
          {
            FloatType* redundant_sol = redundant_sols.data() + 8 * s;
            solution_set.insert(end(solution_set), redundant_sol, redundant_sol + 8);  // If good then add to solution set
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

template<typename FloatType>
bool OPWRailedKinematics<FloatType>::fkAt(const Eigen::Matrix<FloatType, 2, 1>& rail_pose,
                                          const std::vector<FloatType>& pose, Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  solution = opw_kinematics::forward<FloatType>(params_, pose.data());
  Eigen::Transform<FloatType, 3, Eigen::Isometry> rail_tf = Eigen::Transform<FloatType, 3, Eigen::Isometry>::Identity();
  rail_tf.translation().head(2) = rail_pose;
  solution = world_to_rail_base_ * rail_tf * rail_base_to_robot_base_ * solution * tool0_to_tip_;
  return true;
}

template<typename FloatType>
bool OPWRailedKinematics<FloatType>::fk(const FloatType* pose,
                                        Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  Eigen::Matrix<FloatType, 2, 1> rail_pose;
  rail_pose(0) = pose[0];
  rail_pose(1) = pose[1];

  std::vector<FloatType> opw_pose = {pose[2], pose[3], pose[4], pose[5], pose[6], pose[7]};

  return fkAt(rail_pose, opw_pose, solution);
}

template<typename FloatType>
void OPWRailedKinematics<FloatType>::analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry> &p) const
{
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "AnalyzeIK: ", ";");
  std::stringstream ss;
  ss << p.matrix().format(CommaInitFmt);
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  std::string is_valid_fn_defined = "\tIs Valid Function: " ? "True" : "False";
  CONSOLE_BRIDGE_logInform(is_valid_fn_defined.c_str());
  std::string is_redundant_fn_defined = "\tGet Redundant Solutions Function: " ? "True" : "False";
  CONSOLE_BRIDGE_logInform(is_redundant_fn_defined.c_str());

  std::vector<FloatType> solution_set;
  ik(p, nullptr, nullptr, solution_set);
  ss.clear();
  ss << "\tSampling without functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, is_valid_fn_, nullptr, solution_set);
  ss.clear();
  ss << "\tSampling with only IsValid functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, nullptr, redundant_sol_fn_, solution_set);
  ss.clear();
  ss << "\tSampling with only Redundant Solutions functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
  ss.clear();
  ss << "\tSampling with both functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());
}

} // namespace descartes_light

#endif // DESCARTES_OPW_IMPL_RAILED_KINEMATICS_OPW_HPP
