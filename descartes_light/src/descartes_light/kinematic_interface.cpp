#include "descartes_light/kinematic_interface.h"
#include <opw_kinematics/opw_utilities.h>
#include <iostream>

descartes_light::KinematicsInterface::KinematicsInterface(const opw_kinematics::Parameters<double>& params,
                                                          const Eigen::Isometry3d& world_to_base,
                                                          const Eigen::Isometry3d& tool0_to_tip)
  : params_(params)
  , world_to_base_(world_to_base)
  , tool0_to_tip_(tool0_to_tip)
{
}

bool descartes_light::KinematicsInterface::ik(const Eigen::Isometry3d& p, std::vector<double>& solution_set) const
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

      // TODO: Joint limits?
      // If good then add to solution set
      solution_set.insert(end(solution_set), sol, sol + 6);
    }
  }

  return !solution_set.empty();
}

bool descartes_light::KinematicsInterface::fk(const double* pose, Eigen::Isometry3d& solution) const
{
  solution = opw_kinematics::forward<double>(params_, pose);
  solution = world_to_base_ * solution * tool0_to_tip_.inverse();
  return true;
}
