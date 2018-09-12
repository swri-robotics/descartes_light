#include <iostream>
#include <descartes_light/descartes_light.h>
#include <opw_kinematics/opw_parameters_examples.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <descartes_light/collision_checker.h>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <tesseract_ros/kdl/kdl_env.h>

namespace
{

static tesseract::BasicEnvPtr loadEnvironment()
{
  ros::NodeHandle nh; // We're going to load the model from the parameter server...
  std::string urdf_string, srdf_string;

  const std::string robot_description ("robot_description");

  if (!nh.getParam(robot_description, urdf_string)) return nullptr;
  if (!nh.getParam(robot_description + "_semantic", srdf_string)) return nullptr;

  // BUILD URDF & SRDF MODELS //
  auto urdf_model = urdf::parseURDF(urdf_string);
  if (!urdf_model) return nullptr;

  srdf::ModelSharedPtr srdf_model = srdf::ModelSharedPtr(new srdf::Model);
  if (!srdf_model->initString(*urdf_model, srdf_string)) return nullptr;

  // TESSERACT //
  tesseract::tesseract_ros::KDLEnvPtr kdl_env =
      tesseract::tesseract_ros::KDLEnvPtr(new tesseract::tesseract_ros::KDLEnv);
  if (!kdl_env->init(urdf_model, srdf_model)) return nullptr;

  return kdl_env;
}

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

std::vector<descartes_light::PositionSamplerPtr> makePath(descartes_light::CollisionInterfacePtr coll_env)
{
  // The current setup requires that our cartesian sampler is aware of the robot
  // kinematics
  opw_kinematics::Parameters<double> kin_params = opw_kinematics::makeIrb2400_10<double>();
  descartes_light::KinematicsInterface kin_interface (kin_params,
                                                      Eigen::Isometry3d::Identity(),
                                                      Eigen::Isometry3d::Identity());

  Eigen::Isometry3d reference = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -1.0, 0.5) *
                                Eigen::AngleAxisd(M_PI * 0.75, Eigen::Vector3d::UnitY());

  std::vector<descartes_light::PositionSamplerPtr> result;
  for (int i = 0; i < 100 * 2; ++i)
  {
    result.push_back(
          std::make_shared<descartes_light::AxialSymmetricSampler>(
            reference * Eigen::Translation3d(0, i * 0.01, 0), kin_interface, 0.1,
            std::shared_ptr<descartes_light::CollisionInterface>(coll_env->clone())));
  }

  return result;
}

} // anon ns

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  // Load our env
  auto env_ptr = loadEnvironment();
  if (!env_ptr) return 1;

  auto collision_checker = std::make_shared<descartes_light::TesseractCollision>(env_ptr);

  // Define our vertex samplers
  ros::WallTime t1 = ros::WallTime::now();
  const auto path = makePath(collision_checker);
  ros::WallTime t2 = ros::WallTime::now();
  ROS_ERROR_STREAM("DELTA T: " << (t2 - t1).toSec());

  // What logic to connect edges?
  auto edge_eval = std::make_shared<descartes_light::DistanceEdgeEvaluator>(std::vector<double>(6, 1.1));

  descartes_light::Solver graph_builder (6);
  const static double time_per_point = 0.25;
  if (!graph_builder.build(path,
                           std::vector<descartes_core::TimingConstraint>(path.size(), time_per_point),
                           edge_eval))
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
    pt.time_from_start = ros::Duration(i * time_per_point);
    trajectory.points.push_back(pt);
  }

  executeTrajectory(trajectory);

  return 0;
}
