#include "hybrid_planning_common/simple_hybrid_planner.h"
#include "hybrid_planning_common/conversions.h"
#include "descartes_light/descartes_light.h"

namespace
{

std::vector<descartes_core::TimingConstraint> makeTiming(const hybrid_planning_common::PathDefinition& path)
{
  std::vector<descartes_core::TimingConstraint> timing;

  for (const auto& pass : path.path)
  {
    std::vector<descartes_core::TimingConstraint> this_timing (pass.size(), 0.5); // TODO(jmeyer): Compute the real time
    // In Descartes land, the timing constraint represents how long the dt is between the previous point and the point
    // associated with this particular constraint. In a trajectory with only one pass the first point is meaningless (?).
    // Here I want to append many passes together so setting the DT to 0.0 is sort of saying: "Hey, take as long
    // as you need to get to here from the last point".
    if (!this_timing.empty()) this_timing.front() = 0.0;
    timing.insert(end(timing), begin(this_timing), end(this_timing));
  }

  return timing;
}

std::vector<descartes_light::PositionSamplerPtr>
flatten(const std::vector<std::vector<descartes_light::PositionSamplerPtr>>& samplers)
{
  std::vector<descartes_light::PositionSamplerPtr> result;
  for (const auto& inner : samplers)
  {
    result.insert(end(result), begin(inner), end(inner));
  }
  return result;
}

std::pair<trajectory_msgs::JointTrajectory, trajectory_msgs::JointTrajectory>
splitTrajectory(const trajectory_msgs::JointTrajectory& input, std::size_t splice_at_idx) // exclusive
{
  std::pair<trajectory_msgs::JointTrajectory, trajectory_msgs::JointTrajectory> result;
  result.first.header = result.second.header = input.header;
  result.first.joint_names = result.second.joint_names = input.joint_names;

  result.first.points.assign(input.points.begin(), input.points.begin() + splice_at_idx);
  result.second.points.assign(input.points.begin() + splice_at_idx, input.points.end());

  if (!result.second.points.empty())
  {
    const auto start_tm = result.second.points.front().time_from_start;
    for (auto& pt : result.second.points) pt.time_from_start -= start_tm;
  }

  return result;
}

hybrid_planning_common::JointPath splitTrajectory(const trajectory_msgs::JointTrajectory& input,
                                                  const hybrid_planning_common::ToolPath& path)
{
  hybrid_planning_common::JointPath split_path (path.size());

  trajectory_msgs::JointTrajectory working_traj = input;
  for (std::size_t i = 0; i < split_path.size(); ++i)
  {
    auto split = splitTrajectory(working_traj, path[i].size());
    split_path[i] = split.first;
    working_traj = split.second;
  }
  return split_path;
}

bool runDescartes(const hybrid_planning_common::EnvironmentDefinition& env,
                  const hybrid_planning_common::PathDefinition& path,
                  const hybrid_planning_common::SamplerConfiguration& samplers,
                  hybrid_planning_common::JointPath& out)
{
  const auto flat_path = hybrid_planning_common::flatten(path.path);
  const auto timing = makeTiming(path);

  const auto dof = env.environment->getManipulator(env.group_name)->numJoints();
  const auto edge_computer = std::make_shared<descartes_light::DistanceEdgeEvaluator>(std::vector<double>(dof, 1.1));

  descartes_light::Solver graph_builder (dof);
  if (!graph_builder.build(flatten(samplers.samplers), timing, edge_computer))
  {
    std::cerr << "Failed to build vertices\n";
    return false;
  }

  // Search for edges
  std::vector<double> solution;
  if (!graph_builder.search(solution))
  {
    std::cerr << "Search for graph completion failed\n";
    return false;
  }

  auto traj = hybrid_planning_common::descartesToJointTrajectory(
                solution, env.environment->getManipulator(env.group_name)->getJointNames(), ros::Duration(0.5));

  out = splitTrajectory(traj, path.path);

  return true;
}

bool runOptimizer(const hybrid_planning_common::EnvironmentDefinition& env,
                  const hybrid_planning_common::PathDefinition& path,
                  const hybrid_planning_common::OptimizationConfiguration& optimizer_config,
                  const hybrid_planning_common::JointPath& seed,
                  hybrid_planning_common::JointPath& out)
{
  hybrid_planning_common::JointPath optimized_path (path.path.size());

  const auto& joint_names = env.environment->getManipulator(env.group_name)->getJointNames();

  for (std::size_t i = 0; i < seed.size(); ++i)
  {
    std::cout << "Optimizing pass " << i << "\n";
    auto opt_problem = optimizer_config.problem_creator(env, path.path[i], seed[i]);

    trajopt::BasicTrustRegionSQP optimizer (opt_problem);
    optimizer.initialize(trajopt::trajToDblVec(opt_problem->GetInitTraj()));

    const auto opt_status = optimizer.optimize();
    if (opt_status != trajopt::OptStatus::OPT_CONVERGED)
    {
      std::cerr << "Optimizer did not converge on pass " << i << "\n";
      return false;
    }

    auto result = trajopt::getTraj(optimizer.x(), opt_problem->GetVars());

    optimized_path[i] = hybrid_planning_common::trajoptToJointTrajectory(result, joint_names, ros::Duration(0.5));
    std::cout << "Finished optimizing pass " << i << "\n";
  }

  out = optimized_path;
  return true;
}

}

hybrid_planning_common::ProblemResult hybrid_planning_common::simpleHybridPlanner(const ProblemDefinition& def)
{
  JointPath initial_trajectory;

  // Try Descartes if the user provided samplers
  if (def.sampler_config)
  {
    if (!runDescartes(def.env, def.path, *def.sampler_config, initial_trajectory))
    {
      return {};
    }
  }

  // If the user did not provide samplers, initialize the trajectory to some seed value
  if (initial_trajectory.empty())
  {

  }

  // If the user provided a way to create optimization problems, build those out
  JointPath optimized_trajectory;
  if (def.optimizer_config)
  {
    if (!runOptimizer(def.env, def.path, *def.optimizer_config, initial_trajectory, optimized_trajectory))
    {
      return {};
    }
  }

  // Package the result
  ProblemResult result;
  result.succeeded = true; // If we got this far, something worked
  if (def.sampler_config)
    result.sampled_traj = initial_trajectory;

  if (def.optimizer_config)
    result.optimized_traj = optimized_trajectory;

  return result;
}
