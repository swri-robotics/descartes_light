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

#include <tesseract/tesseract.h>
#include <tesseract_rosutils/utils.h>

namespace
{
static tesseract::Tesseract::Ptr loadEnvironment()
{
  ros::NodeHandle nh;  // We're going to load the model from the parameter server...
  std::string urdf_string, srdf_string;

  const std::string robot_description("robot_description");

  if (!nh.getParam(robot_description, urdf_string))
    return nullptr;
  if (!nh.getParam(robot_description + "_semantic", srdf_string))
    return nullptr;

  tesseract::Tesseract::Ptr tesseract_ptr = std::make_shared<tesseract::Tesseract>();
  tesseract_scene_graph::ResourceLocatorFn locator = tesseract_rosutils::locateResource;
  if (!tesseract_ptr->init(urdf_string, srdf_string, locator))
    return nullptr;

  return tesseract_ptr;
}

static bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
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

std::vector<descartes_light::PositionSampler<double>::Ptr>
makePath(descartes_light::CollisionInterface<double>::Ptr coll_env,
         const descartes_light::IsValidFn<double>& is_valid_fn,
         const descartes_light::GetRedundantSolutionsFn<double>& get_redundant_sol_fn)
{
  // The current setup requires that our cartesian sampler is aware of the robot
  // kinematics
  opw_kinematics::Parameters<double> kin_params = opw_kinematics::makeIrb2400_10<double>();
  descartes_light::KinematicsInterface<double>::Ptr kin_interface =
      std::make_shared<descartes_light::OPWKinematics<double>>(
          kin_params, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), is_valid_fn, get_redundant_sol_fn);

  Eigen::Isometry3d reference = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -1.0, 0.5) *
                                Eigen::AngleAxisd(M_PI * 0.75, Eigen::Vector3d::UnitY());

  std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> result;
  for (int i = 0; i < 100 * 2; ++i)
  {
    result.push_back(std::make_shared<descartes_light::AxialSymmetricSampler<double>>(
        reference * Eigen::Translation3d(0, i * 0.01, 0),
        kin_interface,
        0.1,
        descartes_light::CollisionInterface<double>::Ptr(coll_env->clone())));
  }

  return result;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  // Load our env
  auto tesseract_ptr = loadEnvironment();
  if (!tesseract_ptr)
    return 1;

  const auto group_name = "manipulator";

  auto kin_ptr = tesseract_ptr->getFwdKinematicsManager()->getFwdKinematicSolver(group_name);
  if (!kin_ptr)
    return 1;

  tesseract_collision::ContinuousContactManagerPtr manager = tesseract_ptr->getEnvironmentConst()->getContinuousContactManager();
  tesseract_environment::AdjacencyMapPtr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(tesseract_ptr->getEnvironmentConst()->getSceneGraph(),
                                                                                        kin_ptr->getLinkNames(),
                                                                                        tesseract_ptr->getEnvironmentConst()->getCurrentState()->transforms);

  auto collision_checker = std::make_shared<descartes_light::TesseractCollision<double>>(
      tesseract_ptr->getEnvironment(), adjacency_map->getActiveLinkNames(), kin_ptr->getJointNames());

  descartes_light::IsValidFn<double> is_within_limits_fn =
      std::bind(&descartes_light::isWithinLimits<double>, std::placeholders::_1, kin_ptr->getLimits());
  descartes_light::GetRedundantSolutionsFn<double> get_redundant_sol_fn =
      std::bind(&descartes_light::getOPWRedundantSolutions<double>, std::placeholders::_1, kin_ptr->getLimits());

  // Define our vertex samplers
  ros::WallTime t1 = ros::WallTime::now();
  const auto path = makePath<FloatType>(collision_checker, is_within_limits_fn, get_redundant_sol_fn);
  ros::WallTime t2 = ros::WallTime::now();
  ROS_ERROR_STREAM("DELTA T: " << (t2 - t1).toSec());

  // What logic to connect edges?
  auto edge_eval = std::make_shared<descartes_light::DistanceEdgeEvaluator<FloatType>>(std::vector<FloatType>(6, static_cast<FloatType>(1.1)));

  descartes_light::Solver<double> graph_builder(6);
  const static double time_per_point = 0.25;
  if (!graph_builder.build(
          path, std::vector<descartes_core::TimingConstraint<double>>(path.size(), time_per_point), edge_eval))
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
  trajectory.joint_names = tesseract_ptr->getFwdKinematicsManager()->getFwdKinematicSolver(group_name)->getJointNames();
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
