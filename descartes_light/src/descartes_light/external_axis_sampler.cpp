#include "descartes_light/external_axis_sampler.h"
#include <iostream>

descartes_light::ExternalAxisSampler::ExternalAxisSampler(const Eigen::Isometry3d& tool_in_positioner,
                                                          const KinematicsInterface& robot_kin,
                                                          const CollisionInterfacePtr collision)
  : tool_pose_(tool_in_positioner)
  , kin_(robot_kin)
  , collision_(std::move(collision))
{
}

bool descartes_light::ExternalAxisSampler::sample(std::vector<double>& solution_set)
{
  // We need to translate the tool pose to the "robot" frame
  // We need some strategy for moving the positioner around to generate many of these frames
  //    - In the simple case, we can sample the positioner limits evenly but this will often
  //      lead to terrible performance

  auto to_robot_frame = [] (const Eigen::Isometry3d& pose_in_positioner, const double positioner_angle)
  {
    return Eigen::Translation3d(1.25, 0, 0) * Eigen::AngleAxisd(positioner_angle, Eigen::Vector3d::UnitZ()) *
           pose_in_positioner;
  };

  // So we just loop
  const static double discretization = M_PI / 36.;
  for (double angle = -M_PI; angle <= M_PI; angle += discretization)
  {
    std::vector<double> buffer;
    kin_.ik(to_robot_frame(tool_pose_, angle), buffer);

    // Now test the solutions
    const auto n_sols = buffer.size() / 6;
    for (std::size_t i = 0; i < n_sols; ++i)
    {
      const auto* sol_data = buffer.data() + i * 6;
      if (isCollisionFree(sol_data))
      {
        solution_set.insert(end(solution_set), sol_data, sol_data + 6);
        solution_set.insert(end(solution_set), angle);
      }
    }
  }

  return !solution_set.empty();
}

bool descartes_light::ExternalAxisSampler::isCollisionFree(const double* vertex)
{
  return collision_->validate(vertex, 7);
}
