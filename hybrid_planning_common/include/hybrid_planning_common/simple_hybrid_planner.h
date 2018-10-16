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
  ToolPath path;
  double speed;
};


struct SamplerConfiguration
{
  std::vector<std::vector<descartes_light::PositionSamplerPtr>> samplers;
};

struct OptimizationConfiguration
{

  using TrajoptProblemCreator =
    std::function<trajopt::TrajOptProbPtr(const hybrid_planning_common::EnvironmentDefinition&,
                                          const hybrid_planning_common::ToolPass&,
                                          const JointPass&)>;
  TrajoptProblemCreator problem_creator;
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
  bool succeeded = false;
  boost::optional<JointPath> sampled_traj;
  boost::optional<JointPath> optimized_traj;
};

ProblemResult simpleHybridPlanner(const ProblemDefinition& def);

}

#endif // SIMPLE_HYBRID_PLANNER_H
