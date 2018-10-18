#include <hybrid_planning_common/conversions.h>
#include <hybrid_planning_common/load_environment.h>
#include <hybrid_planning_common/path_execution.h>
#include <hybrid_planning_common/path_types.h>
#include <hybrid_planning_common/path_visualization.h>
#include <hybrid_planning_common/abb_4600_kinematic_params.h>
#include <hybrid_planning_common/simple_hybrid_planner.h>

// Descartes
#include <descartes_light/position_sampler.h>
#include <descartes_light/external_axis_sampler.h>

#include <ros/ros.h>

using hybrid_planning_common::ToolPath;
using hybrid_planning_common::ToolPass;
using hybrid_planning_common::makeIrb4600_205_60;

ToolPath makePath()
{
  // Define a path in the frame of the positioner
  // The positioner structure is a cylinder of radius 0.25 meters
  ToolPass pass;

  // put a circle on the top plane
  double s = -0.25;
  for (double r = 0.0; r < 4 * M_PI; r += M_PI / 12)
  {
    Eigen::Isometry3d point = Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitZ()) *
                              Eigen::Translation3d(0, 0.26, s) * Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    s += 0.015;
    pass.push_back(point);
  }

  return {pass};
}

std::vector<std::vector<descartes_light::PositionSamplerPtr>>
makeSamplers(const ToolPath& path, descartes_light::CollisionInterfacePtr coll_env)
{
  // The current setup requires that our cartesian sampler is aware of the robot
  // kinematics
  opw_kinematics::Parameters<double> kin_params = makeIrb4600_205_60<double>();
  const auto tip_to_tool = hybrid_planning_common::torchTool0ToTCP();
  const auto world_to_base = Eigen::Isometry3d::Identity();

  descartes_light::KinematicsInterface kin_interface (kin_params, world_to_base, tip_to_tool);

  std::vector<std::vector<descartes_light::PositionSamplerPtr>> result (path.size());

  for (std::size_t i = 0; i < path.size(); ++i)
  {
    const auto& pass = path[i];
    for (const auto& pose : pass)
    {
      auto collision_clone = descartes_light::CollisionInterfacePtr(coll_env->clone());
      auto sampler = std::make_shared<descartes_light::SpoolSampler>(pose, kin_interface, collision_clone);
      result[i].push_back(std::move(sampler));
    }
  }

  return result;
}

trajopt::TrajOptProbPtr makeProblem(const hybrid_planning_common::EnvironmentDefinition& env,
                                    const hybrid_planning_common::ToolPass& pass,
                                    const hybrid_planning_common::JointPass& seed)
{
  trajopt::ProblemConstructionInfo pci (env.environment);

  // Populate Basic Info
  pci.basic_info.n_steps = pass.size();
  pci.basic_info.manip = env.group_name;
  pci.basic_info.start_fixed = false;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);
  const auto dof = pci.kin->numJoints();

  pci.init_info.type = trajopt::InitInfo::GIVEN_TRAJ;
  pci.init_info.data = hybrid_planning_common::jointTrajectoryToTrajopt(seed);

  // Populate Cost Info
  auto jv = std::make_shared<trajopt::JointVelCostInfo>();
  jv->coeffs = std::vector<double>(dof, 1.0);
  jv->coeffs.back() = 0.0;
  jv->name = "joint_vel";
  jv->term_type = trajopt::TT_COST;
  pci.cost_infos.push_back(jv);

  auto ja = std::make_shared<trajopt::JointAccCostInfo>();
  ja->coeffs = std::vector<double>(dof, 10.0);
  ja->name = "joint_acc";
  ja->term_type = trajopt::TT_COST;
  pci.cost_infos.push_back(ja);

  auto collision = std::make_shared<trajopt::CollisionCostInfo>();
  collision->name = "collision";
  collision->term_type = trajopt::TT_COST;
  collision->continuous = false;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  collision->gap = 1;
  collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.015, 10);

  // Apply a special cost between the sander_disks and the part
  for (auto& c : collision->info)
  {
    c->SetPairSafetyMarginData("welder_tip", "positioner", 0.005, 10.0);
  }

  pci.cost_infos.push_back(collision);

  // Populate Constraints
  for (std::size_t i = 0; i < pass.size(); ++i)
  {
    auto pose = std::make_shared<trajopt::PoseCostInfo>();
    pose->term_type = trajopt::TT_CNT;
    pose->name = "waypoint_cart_" + std::to_string(i);
    pose->link = "positioner";
    pose->target = "welder_tcp";
    pose->timestep = i;
    pose->tcp = pass[i];
    pose->pos_coeffs = Eigen::Vector3d(1, 1, 1);
    pose->rot_coeffs = Eigen::Vector3d(1, 1, 0);
    pci.cnt_infos.push_back(pose);
  }

  return trajopt::ConstructProblem(pci);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "workcell1_demo");
  ros::NodeHandle pnh;
  ros::AsyncSpinner spinner (1); spinner.start();

  // Stage 1: Load & Prepare Environment
  tesseract::tesseract_ros::KDLEnvPtr env;
  if (!hybrid_planning_common::loadEnvironment(env))
  {
    return 1;
  }

  const std::string group_name = "manipulator_positioner";

  hybrid_planning_common::EnvironmentDefinition env_def;
  env_def.environment = env;
  env_def.group_name = group_name;

  // Stage 2: Define the problem
  hybrid_planning_common::PathDefinition path_def;
  path_def.path = makePath();
  path_def.speed = 0.2;

  // Visualize
  hybrid_planning_common::Republisher<geometry_msgs::PoseArray> pose_pub
      ("poses", hybrid_planning_common::toPoseArray(hybrid_planning_common::flatten(path_def.path), "positioner"),
       ros::Rate(10));

  hybrid_planning_common::SamplerConfiguration sampler_config;
  auto collision_iface =
      std::make_shared<descartes_light::TesseractCollision>(env_def.environment, env_def.group_name);
  sampler_config.samplers = makeSamplers(path_def.path, collision_iface);

  hybrid_planning_common::OptimizationConfiguration optimizer_config;
  optimizer_config.problem_creator = makeProblem;

      // Stage 3: Apply the solvers
  hybrid_planning_common::ProblemDefinition problem;
  problem.env = env_def;
  problem.path = path_def;
  problem.sampler_config = sampler_config;
  problem.optimizer_config = optimizer_config;
  hybrid_planning_common::ProblemResult result = hybrid_planning_common::simpleHybridPlanner(problem);


  // Stage 4: Visualize the results
  if (result.succeeded)
  {
    if (result.sampled_traj)
    {
      std::cout << "Displaying sampled trajectory...\n";
      hybrid_planning_common::executeTrajectory(*result.sampled_traj);
      std::cout << "Sampled trajectory done\n";
    }

    if (result.optimized_traj)
    {
      std::cout << "Displaying optimized trajectory...\n";
      hybrid_planning_common::executeTrajectory(*result.optimized_traj);
      std::cout << "Optimized trajectory done\n";
    }
  }

  ros::waitForShutdown();
}
