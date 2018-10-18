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

descartes_light::RailedKinematicsInterface::RailedKinematicsInterface(const opw_kinematics::Parameters<double> &params,
                                                                      const Eigen::Isometry3d &world_to_base,
                                                                      const Eigen::Isometry3d &tool0_to_tip)
  : params_(params)
  , world_to_base_(world_to_base)
  , tool0_to_tip_(tool0_to_tip)
{
}

bool descartes_light::RailedKinematicsInterface::ik(const Eigen::Isometry3d &p, std::vector<double> &solution_set) const
{
  const Eigen::Vector2d rail_lower_limit (-2.0, -2.0);
  const Eigen::Vector2d rail_upper_limit (2.0, 2.0);

  const Eigen::Vector2d origin (p.translation().x(), p.translation().y());

  const double res = 0.4;
  const double start_x = origin.x() - 2.0;
  const double end_x = origin.x() + 2.0;
  const double start_y = origin.y() - 2.0;
  const double end_y = origin.y() + 2.0;

  for (double x = start_x; x < end_x; x += res)
  {
    if (x < rail_lower_limit.x() || x > rail_upper_limit.x()) continue;

    for (double y = start_y; y < end_y; y += res)
    {
      if (y < rail_lower_limit.y() || y > rail_upper_limit.y()) continue;
      else
        ikAt(p, Eigen::Vector2d(x, y), solution_set);
    }
  }

  return !solution_set.empty();
}

bool descartes_light::RailedKinematicsInterface::ikAt(const Eigen::Isometry3d &p, const Eigen::Vector2d &rail_pose,
                                                      std::vector<double> &solution_set) const
{
  const Eigen::Isometry3d in_robot = Eigen::Translation3d(-rail_pose.x(), rail_pose.y(), 0.0) *
                                     world_to_base_.inverse() * p * tool0_to_tip_.inverse();

  std::array<double, 6*8> sols;
  opw_kinematics::inverse(params_, in_robot, sols.data());

  // Check the output
  for (int i = 0; i < 8; i++)
  {
    double* sol = sols.data() + 6 * i;
    if (opw_kinematics::isValid(sol))
    {
      opw_kinematics::harmonizeTowardZero(sol); // Modifies 'sol' in place
      solution_set.insert(end(solution_set), rail_pose.data(), rail_pose.data() + 2); // Insert the X-Y pose of the rail
      solution_set.insert(end(solution_set), sol, sol + 6); // And then insert the robot arm configuration
    }
  }

  return !solution_set.empty();
}
