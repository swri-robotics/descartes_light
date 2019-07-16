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
#ifndef DESCARTES_OPW_RAILED_KINEMATICS_OPW_H
#define DESCARTES_OPW_RAILED_KINEMATICS_OPW_H

#include <descartes_light/interface/kinematics_interface.h>
#include "descartes_light/utils.h"
#include <opw_kinematics/opw_kinematics.h>

namespace descartes_light
{

template<typename FloatType>
class OPWRailedKinematics : public KinematicsInterface<FloatType>
{
public:
  OPWRailedKinematics(const opw_kinematics::Parameters<FloatType>& params,
                      const Eigen::Transform<FloatType, 3, Eigen::Isometry> &world_to_rail_base,
                      const Eigen::Transform<FloatType, 3, Eigen::Isometry> &rail_base_to_robot_base,
                      const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool0_to_tip,
                      const Eigen::Matrix<FloatType, 2, 2>& rail_limits,
                      const Eigen::Matrix<FloatType, 2, 1>& rail_sample_resolution,
                      const FloatType robot_reach,
                      const IsValidFn<FloatType>& is_valid_fn,
                      const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn);

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          std::vector<FloatType>& solution_set) const override;
  bool fk(const FloatType* pose,
          Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const override;

  bool ikAt(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
            const Eigen::Matrix<FloatType, 2, 1>& rail_pose,
            std::vector<FloatType>& solution_set) const;
  bool fkAt(const Eigen::Matrix<FloatType, 2, 1>& rail_pose,
            const std::vector<FloatType>& pose,
            Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const;

  void analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const override;

private:
  opw_kinematics::Parameters<FloatType> params_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_rail_base_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> rail_base_to_robot_base_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool0_to_tip_;
  Eigen::Matrix<FloatType, 2, 2> rail_limits_;
  Eigen::Matrix<FloatType, 2, 1> rail_sample_resolution_;
  FloatType robot_reach_;
  IsValidFn<FloatType> is_valid_fn_;
  GetRedundantSolutionsFn<FloatType> redundant_sol_fn_;

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          const IsValidFn<FloatType>& is_valid_fn,
          const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
          std::vector<FloatType>& solution_set) const;

  bool ikAt(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
            const Eigen::Matrix<FloatType, 2, 1>& rail_pose,
            const IsValidFn<FloatType>& is_valid_fn,
            const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
            std::vector<FloatType>& solution_set) const;
};

using OPWRailedKinematicsF = OPWRailedKinematics<float>;
using OPWRailedKinematicsD = OPWRailedKinematics<double>;

} // namespace descartes_light

#endif // DESCARTES_OPW_RAILED_KINEMATICS_OPW_H
