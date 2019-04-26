#include <iostream>
#include <functional>

#include <opw_kinematics/opw_parameters_examples.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <descartes_light/descartes_light.h>
#include <descartes_tesseract/descartes_tesseract_collision_checker.h>
#include <descartes_samplers/samplers/axial_symmetric_sampler.h>
#include <descartes_samplers/evaluators/distance_edge_evaluator.h>
#include <descartes_opw/descartes_opw_kinematics.h>
#include <descartes_opw/utils.h>

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

template<typename FloatType>
std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> makePath(typename descartes_light::CollisionInterface<FloatType>::Ptr coll_env,
                                                                                const descartes_light::IsValidFn<FloatType>& is_valid_fn,
                                                                                const descartes_light::GetRedundantSolutionsFn<FloatType>& get_redundant_sol_fn)
{
  // The current setup requires that our cartesian sampler is aware of the robot
  // kinematics
  opw_kinematics::Parameters<FloatType> kin_params = opw_kinematics::makeIrb2400_10<FloatType>();
  typename descartes_light::KinematicsInterface<FloatType>::Ptr kin_interface = std::make_shared<descartes_light::OPWKinematics<FloatType>>(kin_params, Eigen::Isometry3d::Identity().cast<FloatType>(), Eigen::Isometry3d::Identity().cast<FloatType>(), is_valid_fn, get_redundant_sol_fn);

  Eigen::Isometry3d reference = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -1.0, 0.5) *
                                Eigen::AngleAxisd(M_PI * 0.75, Eigen::Vector3d::UnitY());

  std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> result;
  for (int i = 0; i < 100 * 2; ++i)
  {
    Eigen::Isometry3d position = reference * Eigen::Translation3d(0, static_cast<double>(i) * 0.01, 0);
    result.push_back(
          std::make_shared<descartes_light::AxialSymmetricSampler<FloatType>>(
            position.cast<FloatType>(), kin_interface, static_cast<FloatType>(0.1),
            typename descartes_light::CollisionInterface<FloatType>::Ptr(coll_env->clone())));
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
  if (!env_ptr)
  {
    ROS_ERROR_STREAM("Failed to get environment");
    return -1;
  }

  const auto group_name = "manipulator";

  auto kin_ptr = env_ptr->getManipulator(group_name);
  if (!kin_ptr)
  {
    ROS_ERROR_STREAM("Failed to kinematics manipulator");
    return -1;
  }

  // Declare what type of float to use
  using FloatType = double;

  auto collision_checker = std::make_shared<descartes_light::TesseractCollision<FloatType>>(env_ptr, kin_ptr->getLinkNames(), kin_ptr->getJointNames());

  descartes_light::IsValidFn<FloatType> is_within_limits_fn = std::bind(&descartes_light::isWithinLimits<FloatType>, std::placeholders::_1, kin_ptr->getLimits().cast<FloatType>());
  descartes_light::GetRedundantSolutionsFn<FloatType> get_redundant_sol_fn = std::bind(&descartes_light::getOPWRedundantSolutions<FloatType>, std::placeholders::_1, kin_ptr->getLimits().cast<FloatType>());

  // Define our vertex samplers
  ros::WallTime t1 = ros::WallTime::now();
  const auto path = makePath<FloatType>(collision_checker, is_within_limits_fn, get_redundant_sol_fn);
  ros::WallTime t2 = ros::WallTime::now();
  ROS_ERROR_STREAM("DELTA T: " << (t2 - t1).toSec());

  // What logic to connect edges?
  auto edge_eval = std::make_shared<descartes_light::DistanceEdgeEvaluator<FloatType>>(std::vector<FloatType>(6, static_cast<FloatType>(1.1)));

  descartes_light::Solver<FloatType> graph_builder (6);
  const static FloatType time_per_point = static_cast<FloatType>(0.25);
  if (!graph_builder.build(path,
                           std::vector<descartes_core::TimingConstraint<FloatType>>(path.size(), time_per_point),
                           edge_eval))
  {
    ROS_ERROR_STREAM("Failed to build graph");
    return -1;
  }

  // Search for edges
  std::vector<FloatType> solution;
  if (!graph_builder.search(solution))
  {
    ROS_ERROR_STREAM("Search for graph completion failed");
    return -1;
  }

  // To joint trajectory
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = env_ptr->getManipulator(group_name)->getJointNames();
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
