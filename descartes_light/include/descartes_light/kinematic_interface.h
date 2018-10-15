#ifndef DESCARTES_LIGHT_KINEMATICS_H
#define DESCARTES_LIGHT_KINEMATICS_H

#include <Eigen/Geometry>
#include <vector>
#include <opw_kinematics/opw_kinematics.h>

namespace descartes_light
{

class KinematicsInterface
{
public:
  KinematicsInterface(const opw_kinematics::Parameters<double>& params,
                      const Eigen::Isometry3d& world_to_base,
                      const Eigen::Isometry3d& tool0_to_tip);

  bool ik(const Eigen::Isometry3d& p, std::vector<double>& solution_set) const;
  bool fk(const double* pose, Eigen::Isometry3d& solution) const;

private:
  opw_kinematics::Parameters<double> params_;
  Eigen::Isometry3d world_to_base_;
  Eigen::Isometry3d tool0_to_tip_;
};

class RailedKinematicsInterface
{
public:
  RailedKinematicsInterface(const opw_kinematics::Parameters<double>& params,
                            const Eigen::Isometry3d& world_to_base,
                            const Eigen::Isometry3d& tool0_to_tip);

  bool ik(const Eigen::Isometry3d& p, std::vector<double>& solution_set) const;

  bool ikAt(const Eigen::Isometry3d& p, const Eigen::Vector2d& rail_pose, std::vector<double>& solution_set) const;

private:
  opw_kinematics::Parameters<double> params_;
  Eigen::Isometry3d world_to_base_;
  Eigen::Isometry3d tool0_to_tip_;
};

}

#endif
