#ifndef DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_OMPL_SOLVER_HPP
#define DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_OMPL_SOLVER_HPP

#include <descartes_light/ompl/ompl_solver.h>
#include <descartes_light/ompl/descartes_space.h>

#include <descartes_light/descartes_macros.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>

DESCARTES_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
DESCARTES_IGNORE_WARNINGS_POP

static void reportFailedVertices(const std::vector<std::size_t>& indices)
{
  if (indices.empty())
    CONSOLE_BRIDGE_logInform("No failed vertices");
  else
  {
    std::stringstream ss;
    ss << "Failed vertices:\n";
    for (const auto& i : indices)
      ss << "\t" << i << "\n";

    CONSOLE_BRIDGE_logWarn(ss.str().c_str());
  }
}

namespace descartes_light
{
template <typename FloatType>
BuildStatus LadderGraphOMPLSolver<FloatType>::buildImpl(
    const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
    const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_evaluators,
    const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_evaluators)
{
  BuildStatus status;
  graph_.resize(trajectory.size());

  // Build Vertices
  long num_waypoints = static_cast<long>(trajectory.size());
  long cnt = 0;

  using Clock = std::chrono::high_resolution_clock;
  std::chrono::time_point<Clock> start_time = Clock::now();
#pragma omp parallel for num_threads(num_threads_)
  for (long i = 0; i < static_cast<long>(trajectory.size()); ++i)
  {
    std::vector<StateSample<FloatType>> samples = trajectory[static_cast<size_t>(i)]->sample();
    if (!samples.empty())
    {
      auto& r = graph_.getRung(static_cast<size_t>(i));
      r.nodes.reserve(samples.size());
      for (auto& sample : samples)
      {
        if (state_evaluators.empty())
        {
          r.nodes.push_back(Node<FloatType>(sample));
        }
        else
        {
          std::pair<bool, FloatType> results = state_evaluators[static_cast<size_t>(i)]->evaluate(*sample.state);
          if (results.first)
          {
            sample.cost += results.second;
            r.nodes.push_back(Node<FloatType>(sample));
          }
        }
      }
    }
    else
    {
#pragma omp critical
      {
        status.failed_vertices.push_back(static_cast<size_t>(i));
      }
    }
#ifndef NDEBUG
#pragma omp critical
    {
      ++cnt;
      std::stringstream ss;
      ss << "Descartes Processed: " << cnt << " of " << num_waypoints << " vertices";
      CONSOLE_BRIDGE_logInform(ss.str().c_str());
    }
#endif
  }
  double duration = std::chrono::duration<double>(Clock::now() - start_time).count();
  CONSOLE_BRIDGE_logDebug("Descartes took %0.4f seconds to build vertices.", duration);

  // Created modified ladder rungs with added start and end locations to allow for a single start and goal state
  graph_.insertRung(0);
  graph_.insertRung(graph_.size());
  auto& first_rung = graph_.getRung(0);
  first_rung.nodes.reserve(1);
  auto& first_sample = trajectory.front()->sample().front();
  first_rung.nodes.push_back(Node<FloatType>(first_sample));
  auto& last_rung = graph_.getRung(graph_.size() - 1);
  last_rung.nodes.push_back(Node<FloatType>(first_sample));

  // Construct Descartes state space that is to be searched
  dss_ = std::make_shared<descartes_light::DescartesStateSpace<FloatType>>(graph_, edge_evaluators, max_dist_);

  // Set the longest valid segment fraction (This isn't really used)
  dss_->setLongestValidSegmentFraction(0.5);

  // Initialize the simple setup
  ss_ = std::make_shared<ompl::geometric::SimpleSetup>(dss_);

  // Create the start and goal states
  ompl::base::ScopedState<descartes_light::DescartesStateSpace<FloatType>> start(dss_), goal(dss_);
  start->rung = 0;
  start->idx = 0;
  goal->rung = graph_.size() - 1;
  goal->idx = 0;

  // Set the start and goal states
  ss_->setStartAndGoalStates(start, goal);

  // Set state validity checker (always true because only valid states should exist in the ladder graph)
  ss_->setStateValidityChecker([](const ompl::base::State* /*state*/) { return true; });

  // Set motion validator to custom defined motion validator for the Descartes state space
  ss_->getSpaceInformation()->setMotionValidator(
      std::make_shared<descartes_light::DescartesMotionValidator<FloatType>>(ss_->getSpaceInformation()));

  reportFailedVertices(status.failed_vertices);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("LadderGraphSolver failed to build graph.");
  }

  return status;
}

template <typename FloatType>
SearchResult<FloatType> LadderGraphOMPLSolver<FloatType>::ompl_search(std::shared_ptr<ompl::base::Planner> ompl_planner)
{
  // Set planner to use passed through planner
  ss_->setPlanner(ompl_planner);

  // Set up ompl plan
  ss_->setup();

  // Perform the search
  ompl::base::PlannerStatus status = ss_->solve(planning_time_);

  // Make sure search succeeded
  if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
    throw std::runtime_error("OMPL search failed to find a solution through the Descartes Graph");

  // Get solution path
  ompl::geometric::PathGeometric path = ss_->getSolutionPath();

  // Convert solution to a vector of states to be evaluated and extracted
  std::vector<ompl::base::State*> path_states = path.getStates();

  // Initialize total cost to be 0
  FloatType cost = 0;

  // Determine total cost of motion through the solution path
  for (std::size_t i = 0; i < path_states.size() - 1; i++)
  {
    auto curr_state = path_states[i];
    auto next_state = path_states[i + 1];
    double dist = dss_->distance(curr_state, next_state);
    if (dist == dss_->getDistanceEpsilon())
      dist = 0;
    cost += static_cast<FloatType>(dist);
  }

  // Populate the result to be returned
  SearchResult<FloatType> result;
  result.cost = cost;

  // Convert ompl states into a vector of vertex descriptions
  //  std::vector<descartes_light::VertexDesc<FloatType>> vd_path;
  std::size_t rung_offset = 0;
  std::size_t prev_rung = 0;
  result.trajectory.reserve(graph_.size());
  for (std::size_t i = 0; i < graph_.size() - 2; i++)
  {
    std::size_t rung =
        path_states[i + rung_offset]->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->rung;
    std::size_t idx =
        path_states[i + rung_offset]->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->idx;
    // If the rung matches the previous rung then move on to the next vertex.
    // Repeat points have no cost in OMPL, but don't make sense in the context of Descartes.
    if (rung == prev_rung)
    {
      rung_offset++;
      i--;
    }
    else
    {
      result.trajectory.push_back(graph_.getRung(rung).nodes[idx].sample.state);
    }
    prev_rung = rung;
  }

  return result;
}

template <typename FloatType>
SearchResult<FloatType> LadderGraphOMPLRRTSolver<FloatType>::search()
{
  // Define and setup the RRT planner
  std::shared_ptr<ompl::geometric::RRT> planner =
      std::make_shared<ompl::geometric::RRT>((LadderGraphOMPLSolver<FloatType>::ss_->getSpaceInformation()));
  planner->setRange(LadderGraphOMPLSolver<FloatType>::max_dist_);

  // Get the resulting path through the graph
  SearchResult<FloatType> result = descartes_light::LadderGraphOMPLSolver<FloatType>::ompl_search(planner);

  return result;
}

template <typename FloatType>
SearchResult<FloatType> LadderGraphOMPLRRTConnectSolver<FloatType>::search()
{
  // Define and setup the RRT planner
  std::shared_ptr<ompl::geometric::RRTConnect> planner =
      std::make_shared<ompl::geometric::RRTConnect>((LadderGraphOMPLSolver<FloatType>::ss_->getSpaceInformation()));
  planner->setRange(LadderGraphOMPLSolver<FloatType>::max_dist_);

  // Get the resulting path through the graph
  SearchResult<FloatType> result = descartes_light::LadderGraphOMPLSolver<FloatType>::ompl_search(planner);

  return result;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_OMPL_SOLVER_HPP
