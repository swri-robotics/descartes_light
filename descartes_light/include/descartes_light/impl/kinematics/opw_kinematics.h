#ifndef DESCARTES_LIGHT_KINEMATICS_OPW_H
#define DESCARTES_LIGHT_KINEMATICS_OPW_H

#include <descartes_light/core/kinematics_interface.h>
#include "descartes_light/impl/utils.h"
#include <opw_kinematics/opw_kinematics.h>

namespace descartes_light
{

template<typename FloatType>
class OPWKinematics : public KinematicsInterface<FloatType>
{
public:
  OPWKinematics(const opw_kinematics::Parameters<FloatType>& params,
                const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_base,
                const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool0_to_tip,
                const IsValidFn<FloatType>& is_valid_fn,
                const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn);

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          std::vector<FloatType>& solution_set) const override;
  bool fk(const FloatType* pose,
          Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const override;

  void analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const override;

private:
  opw_kinematics::Parameters<FloatType> params_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_base_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool0_to_tip_;
  IsValidFn<FloatType> is_valid_fn_;
  GetRedundantSolutionsFn<FloatType> redundant_sol_fn_;

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          const IsValidFn<FloatType>& is_valid_fn,
          const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
          std::vector<FloatType>& solution_set) const;
};

using OPWKinematicsF = OPWKinematics<float>;
using OPWKinematicsD = OPWKinematics<double>;

} // namespace descartes_light

#endif // DESCARTES_LIGHT_KINEMATICS_OPW_H
