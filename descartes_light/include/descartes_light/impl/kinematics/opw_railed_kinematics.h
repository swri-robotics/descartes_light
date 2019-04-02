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

#include <descartes_light/core/railed_kinematics_interface.h>
#include <opw_kinematics/opw_kinematics.h>

namespace descartes_light
{

class OPWRailedKinematics : public RailedKinematicsInterface
{
public:
  OPWRailedKinematics(const opw_kinematics::Parameters<double>& params,
                      const Eigen::Isometry3d& world_to_base,
                      const Eigen::Isometry3d& tool0_to_tip);

  bool ik(const Eigen::Isometry3d& p, std::vector<double>& solution_set) const override;

  bool ikAt(const Eigen::Isometry3d& p, const Eigen::Vector2d& rail_pose, std::vector<double>& solution_set) const override;

private:
  opw_kinematics::Parameters<double> params_;
  Eigen::Isometry3d world_to_base_;
  Eigen::Isometry3d tool0_to_tip_;
};

}

#endif // DESCARTES_LIGHT_RAILED_KINEMATICS_OPW_H
