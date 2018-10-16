#include <hybrid_planning_common/conversions.h>
#include <hybrid_planning_common/load_environment.h>
#include <hybrid_planning_common/path_execution.h>
#include <hybrid_planning_common/path_types.h>
#include <hybrid_planning_common/path_visualization.h>
#include <hybrid_planning_common/abb_4600_kinematic_params.h>
#include <hybrid_planning_common/simple_hybrid_planner.h>

// Descartes
#include <descartes_light/position_sampler.h>

#include <ros/ros.h>

using hybrid_planning_common::Path;
using hybrid_planning_common::Pass;
using hybrid_planning_common::makeIrb4600_205_60;

Path makePath(bool tilt = true)
{
  Path path;

  // Reference Pose
  auto origin = Eigen::Isometry3d::Identity();
  origin.translation() = Eigen::Vector3d(0.5, 0, 0.5);

  const int n_passes = 10;

  // For each pass
  for (int r = 0; r < n_passes; ++r)
  {
     Pass this_pass;

     // For each pose in the pass
    for (int i = -10; i <= 10; ++i)
    {
      // The last operation flips the pose around so +Z is into the part
      auto p = origin * Eigen::Translation3d(r * 0.1, i * 0.05, 0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
      this_pass.push_back(p);
    }

    // For odd passes, we want the robot to move the other direction (so the robot doesn't "carriage return")
    if (r % 2 != 0)
    {
      std::reverse(this_pass.begin(), this_pass.end());
      // Also keep x along the direction of travel (important when tilting)
      for (auto& p : this_pass) p = p * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    }

    path.push_back(this_pass);
  }

  // If the user wants it, tilt the poses so the tool "digs" into the surface
  const double tilt_angle = -10 * M_PI / 180.;
  if (tilt)
  {
    for (auto& pass : path)
      for (auto& p : pass)
        p = p * Eigen::AngleAxisd(tilt_angle, Eigen::Vector3d::UnitX()) * Eigen::Translation3d(0, 0.05, 0);
  }

  return path;
}

std::vector<std::vector<descartes_light::PositionSamplerPtr>>
makeSamplers(const Path& path, descartes_light::CollisionInterfacePtr coll_env)
{
  // The current setup requires that our cartesian sampler is aware of the robot
  // kinematics
  opw_kinematics::Parameters<double> kin_params = makeIrb4600_205_60<double>();
  auto tip_to_tool = hybrid_planning_common::sanderTool0ToTCP();
  descartes_light::KinematicsInterface kin_interface (kin_params,
                                                      Eigen::Isometry3d::Identity(), tip_to_tool);


  std::vector<std::vector<descartes_light::PositionSamplerPtr>> samplers (path.size());

  // Loop over each pass & each pose and create a sampler specifically for this pose
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    for (const auto& pose : path[i])
    {
      // The tesseract collision interface is not thread safe and we want to run these checks
      // in parallel, so I need to create a seperate checker for each point.
      auto collision_clone = descartes_light::CollisionInterfacePtr(coll_env->clone());

      // Create a sampler that samples around the Z axis of the TCP
      auto sampler = std::make_shared<descartes_light::AxialSymmetricSampler>
          (pose, kin_interface, M_PI/12.0, collision_clone);

      samplers[i].push_back(std::move(sampler));
    }
  }

  return samplers;
}

static trajopt::TrajOptProbPtr makeProblem(const hybrid_planning_common::EnvironmentDefinition& env,
                                           const hybrid_planning_common::Pass& pass,
                                           const trajectory_msgs::JointTrajectory& seed)
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
  jv->coeffs = std::vector<double>(dof, 2.5);
  jv->name = "joint_vel";
  jv->term_type = trajopt::TT_COST;
  pci.cost_infos.push_back(jv);

  auto ja = std::make_shared<trajopt::JointAccCostInfo>();
  ja->coeffs = std::vector<double>(dof, 5.0);
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
  collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 20);

  // Apply a special cost between the sander_disks and the part
//  for (auto& c : collision->info)
//  {
//    c->SetPairSafetyMarginData("sander_disk", "part", -0.01, 20.0);
//    c->SetPairSafetyMarginData("sander_shaft", "part", 0.0, 20.0);
//  }

  pci.cost_infos.push_back(collision);

  auto to_wxyz = [](const Eigen::Isometry3d& p) {
    Eigen::Quaterniond q (p.linear());
    Eigen::Vector4d wxyz;
    wxyz(0) = q.w();
    wxyz(1) = q.x();
    wxyz(2) = q.y();
    wxyz(3) = q.z();
    return wxyz;
  };

  // Populate Constraints
  for (std::size_t i = 0; i < pass.size(); ++i)
  {
    auto pose = std::make_shared<trajopt::StaticPoseCostInfo>();
    pose->term_type = trajopt::TT_CNT;
    pose->name = "waypoint_cart_" + std::to_string(i);
    pose->link = "sander_tcp";
    pose->timestep = i;
    pose->xyz = pass[i].translation();
    pose->wxyz = to_wxyz(pass[i]);
    pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
    pose->rot_coeffs = Eigen::Vector3d(10, 10, 0);
    pci.cnt_infos.push_back(pose);
  }

  return trajopt::ConstructProblem(pci);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "workcell1_demo");
  ros::NodeHandle pnh;

  // Stage 1: Load & Prepare Environment
  tesseract::tesseract_ros::KDLEnvPtr env;
  if (!hybrid_planning_common::loadEnvironment(env))
  {
    return 1;
  }

  const std::string group_name = "manipulator";

  hybrid_planning_common::EnvironmentDefinition env_def;
  env_def.environment = env;
  env_def.group_name = group_name;

  // Stage 2: Define the problem
  hybrid_planning_common::PathDefinition path_def;
  path_def.path = makePath(true);
  path_def.speed = 0.2;

  hybrid_planning_common::SamplerConfiguration sampler_config;
  auto collision_iface =
      std::make_shared<descartes_light::TesseractCollision>(env_def.environment, env_def.group_name);
  sampler_config.samplers = makeSamplers(path_def.path, collision_iface);

  // Stage 3: Apply the solvers
  hybrid_planning_common::ProblemDefinition problem;
  problem.env = env_def;
  problem.path = path_def;
  problem.sampler_config = sampler_config;
  hybrid_planning_common::ProblemResult result = hybrid_planning_common::simpleHybridPlanner(problem);

  // Stage 4: Visualize the results
  if (result.succeeded)
  {
    if (result.sampled_traj)
    {
      hybrid_planning_common::executeTrajectory(*result.sampled_traj);
    }

    if (result.optimized_traj)
    {
      hybrid_planning_common::executeTrajectory(*result.optimized_traj);
    }
  }
}
