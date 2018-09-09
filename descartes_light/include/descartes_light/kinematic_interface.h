#ifndef DESCARTES_LIGHT_KINEMATICS_H
#define DESCARTES_LIGHT_KINEMATICS_H

#include <Eigen/Geometry>

namespace descartes_light
{

class KinematicsInterface
{
public:
  bool ik(const Eigen::Isometry3d& p, std::vector<double>& solution_set) const;
  bool fk(const double* pose, Eigen::Isometry3d& solution) const;
};

}

#endif
