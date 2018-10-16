#ifndef ABB_4600_KINEMATIC_PARAMS_H
#define ABB_4600_KINEMATIC_PARAMS_H

#include <cmath> // for M_PI
#include <Eigen/Geometry>
#include <opw_kinematics/opw_parameters.h>

namespace hybrid_planning_common
{

/**
 * I use a common robot arm for all of these tests, so I dumped its kinematic parameters
 * here.
 */
template <typename T>
inline opw_kinematics::Parameters<T> makeIrb4600_205_60()
{
  opw_kinematics::Parameters<T> p;
  p.a1 = T(0.175);
  p.a2 = T(-0.175);
  p.b =  T(0.000);
  p.c1 = T(0.495);
  p.c2 = T(0.900);
  p.c3 = T(0.960);
  p.c4 = T(0.135);

  p.offsets[2] = -M_PI / 2.0;

  return p;
}

// TCP offset for the sanding tool
inline Eigen::Isometry3d sanderTool0ToTCP()
{
  return Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.235, 0.0, 0.1) *
         Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
}

// TCP offset for the torch / welding tool
inline Eigen::Isometry3d torchTool0ToTCP()
{
  return Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.06, 0.0, 0.2125) *
         Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitY());
}


}

#endif // ABB_4600_KINEMATIC_PARAMS_H
