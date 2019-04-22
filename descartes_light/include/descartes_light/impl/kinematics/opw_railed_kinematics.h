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
#ifndef DESCARTES_LIGHT_RAILED_KINEMATICS_OPW_H
#define DESCARTES_LIGHT_RAILED_KINEMATICS_OPW_H

#include <descartes_light/core/kinematics_interface.h>
#include "descartes_light/impl/utils.h"
#include <opw_kinematics/opw_kinematics.h>

namespace descartes_light
{

class OPWRailedKinematics : public KinematicsInterface
{
public:
  OPWRailedKinematics(const opw_kinematics::Parameters<double>& params,
                      const Eigen::Isometry3d &world_to_rail_base,
                      const Eigen::Isometry3d &rail_base_to_robot_base,
                      const Eigen::Isometry3d& tool0_to_tip,
                      const Eigen::Matrix2d& rail_limits,
                      const Eigen::Vector2d& rail_sample_resolution,
                      const double robot_reach,
                      const IsValidFn& is_valid_fn,
                      const GetRedundentSolutionsFn& redundent_sol_fn);

  bool ik(const Eigen::Isometry3d& p, std::vector<double>& solution_set) const override;
  bool fk(const double* pose, Eigen::Isometry3d& solution) const override;

  bool ikAt(const Eigen::Isometry3d& p, const Eigen::Vector2d& rail_pose, std::vector<double>& solution_set) const;
  bool fkAt(const Eigen::Vector2d& rail_pose, const std::vector<double>& pose, Eigen::Isometry3d& solution) const;

  void analyzeIK(const Eigen::Isometry3d &p) const override;

private:
  opw_kinematics::Parameters<double> params_;
  Eigen::Isometry3d world_to_rail_base_;
  Eigen::Isometry3d rail_base_to_robot_base_;
  Eigen::Isometry3d tool0_to_tip_;
  Eigen::Matrix2d rail_limits_;
  Eigen::Vector2d rail_sample_resolution_;
  double robot_reach_;
  IsValidFn is_valid_fn_;
  GetRedundentSolutionsFn redundent_sol_fn_;

  bool ik(const Eigen::Isometry3d& p,
          const IsValidFn& is_valid_fn,
          const GetRedundentSolutionsFn& redundent_sol_fn,
          std::vector<double>& solution_set) const;

  bool ikAt(const Eigen::Isometry3d& p,
            const Eigen::Vector2d& rail_pose,
            const IsValidFn& is_valid_fn,
            const GetRedundentSolutionsFn& redundent_sol_fn,
            std::vector<double>& solution_set) const;
};

}

#endif // DESCARTES_LIGHT_RAILED_KINEMATICS_OPW_H
