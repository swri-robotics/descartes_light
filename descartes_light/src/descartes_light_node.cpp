#include <iostream>
#include <descartes_light/descartes_light.h>
#include <opw_kinematics/opw_parameters_examples.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>

namespace
{

static bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac ("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    std::cerr << "Could not connect to action server";
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);

  return ac.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
}

std::vector<descartes_light::PositionSamplerPtr> makePath()
{
  // The current setup requires that our cartesian sampler is aware of the robot
  // kinematics
  opw_kinematics::Parameters<double> kin_params = opw_kinematics::makeIrb2400_10<double>();
  descartes_light::KinematicsInterface kin_interface (kin_params,
                                                      Eigen::Isometry3d::Identity(),
                                                      Eigen::Isometry3d::Identity());

  Eigen::Isometry3d reference = Eigen::Isometry3d::Identity() * Eigen::Translation3d(1.0, -1.0, 0.5) *
                                Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

  std::vector<descartes_light::PositionSamplerPtr> result;
  for (int i = 0; i < 10 * 2; ++i)
  {
    result.push_back(
          std::make_shared<descartes_light::CartesianPointSampler>(
            reference * Eigen::Translation3d(0, i * 0.1, 0), kin_interface));
  }

  return result;
}

} // anon ns

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  // Define our vertex samplers
  const auto path = makePath();

  // What logic to connect edges?
  auto edge_eval = std::make_shared<descartes_light::DistanceEdgeEvaluator>();

  descartes_light::Solver graph_builder (6);
  if (!graph_builder.build(path, edge_eval))
  {
    std::cerr << "Failed to build graph\n";
    return 1;
  }

  // Search for edges
  std::vector<double> solution;
  if (!graph_builder.search(solution))
  {
    std::cerr << "Search for graph completion failed\n";
    return 1;
  }

  // To joint trajectory
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  for (std::size_t i = 0; i < solution.size() / 6; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions.assign(solution.begin() + i * 6, solution.begin() + (i + 1) * 6);
    pt.time_from_start = ros::Duration(i * 0.5);
    trajectory.points.push_back(pt);
  }

  executeTrajectory(trajectory);

  return 0;
}
