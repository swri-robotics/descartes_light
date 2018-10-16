#ifndef SIMPLE_HYBRID_PLANNER_H
#define SIMPLE_HYBRID_PLANNER_H

#include <tesseract_core/basic_env.h>
#include <hybrid_planning_common/path_types.h>

#include <trajopt/problem_description.hpp>
#include <descartes_light/position_sampler.h>

#include <trajectory_msgs/JointTrajectory.h>

namespace hybrid_planning_common
{

struct EnvironmentDefinition
{
  tesseract::BasicEnvConstPtr environment;
  std::string group_name;
};

struct PathDefinition
{
  Path path;
  double speed;
};


struct SamplerConfiguration
{
  std::vector<std::vector<descartes_light::PositionSamplerPtr>> samplers;
};

struct OptimizationConfiguration
{
//  std::vector<trajop`t::TrajOptProbPtr> problems;
};

struct ProblemDefinition
{
  EnvironmentDefinition env;
  PathDefinition path;

  boost::optional<SamplerConfiguration> sampler_config;
  boost::optional<OptimizationConfiguration> optimizer_config;
};

// Result
struct ProblemResult
{
  bool succeeded;
  boost::optional<trajectory_msgs::JointTrajectory> sampled_traj;
  boost::optional<trajectory_msgs::JointTrajectory> optimized_traj;
};

ProblemResult simpleHybridPlanner(const ProblemDefinition& def);

}

#endif // SIMPLE_HYBRID_PLANNER_H
