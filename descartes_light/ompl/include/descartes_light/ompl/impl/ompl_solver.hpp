#ifndef DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_OMPL_SOLVER_HPP
#define DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_OMPL_SOLVER_HPP

#include <descartes_light/ompl/ompl_solver.h>
#include <descartes_light/bgl/impl/event_visitors.hpp>
#include <descartes_light/ompl/descartes_space.h>

#include <descartes_light/descartes_macros.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>

namespace descartes_light
{
template <typename FloatType>
void BGLOMPLSolver<FloatType>::initOMPL()
{
  // Convenience aliases
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  const auto& source_ = BGLSolverBase<FloatType>::source_;
  const auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;
  const auto& edge_eval_ = BGLSolverBaseSVDE<FloatType>::edge_eval_;

  // Created modified ladder rungs with added start and end locations to allow for a single start and goal state
  auto mod_ladder_rungs = std::move(ladder_rungs_);
  std::vector<descartes_light::VertexDesc<FloatType>> blank_rung = {source_};
  mod_ladder_rungs.insert(mod_ladder_rungs.begin(), blank_rung);
  mod_ladder_rungs.push_back(blank_rung);


  // Construct Descartes state space that is to be searched
  dss_ = std::make_shared<descartes_light::DescartesStateSpace<FloatType>>(graph_,
                                                                           mod_ladder_rungs,
                                                                           edge_eval_,
                                                                           max_dist_,
                                                                           rung_to_rung_dist_);

  // Set the longest valid segment fraction (This isn't really used)
  dss_->setLongestValidSegmentFraction(0.5);

  // Initialize the simple setup
  ss_ = std::make_shared<ompl::geometric::SimpleSetup>(dss_);

  // Create the start and goal states
  ompl::base::ScopedState<descartes_light::DescartesStateSpace<FloatType>> start(dss_), goal(dss_);
  std::pair<long unsigned int, long unsigned int> start_v(0, 0);
  std::pair<long unsigned int, long unsigned int> goal_v(mod_ladder_rungs.size() - 1, 0);
  start->vertex = start_v;
  goal->vertex = goal_v;

  // Set the start and goal states
  ss_->setStartAndGoalStates(start, goal);

  // Set state validity checker (always true because only valid states should exist in the ladder graph)
  ss_->setStateValidityChecker([](const ompl::base::State* /*state*/)
  {
    return true;
  });

  // Set motion validator to custom defined motion validator for the Descartes state space
  ss_->getSpaceInformation()->setMotionValidator(
        std::make_shared<descartes_light::DescartesMotionValidator<FloatType>>(ss_->getSpaceInformation()));

}

template <typename FloatType>
SearchResult<FloatType> BGLOMPLSolver<FloatType>::ompl_search(std::shared_ptr<ompl::base::Planner> ompl_planner)
{
  // Convenience alias
  const auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;

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
  std::vector<ompl::base::State *> path_states = path.getStates();

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

  // Convert ompl states into a vector of vertex descriptions
  std::vector<descartes_light::VertexDesc<FloatType>> vd_path;
  std::size_t rung_offset = 0;
  std::size_t prev_rung = 0;
  for (std::size_t i = 0; i < ladder_rungs_.size(); i++)
  {
    std::size_t rung = path_states[i+rung_offset]->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.first;
    std::size_t idx = path_states[i+rung_offset]->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.second;
    // If the rung matches the previous rung then move on to the next vertex.
    // Repeat points have no cost in OMPL, but don't make sense in the context of Descartes.
    if (rung == prev_rung)
    {
      rung_offset++;
      i--;
    }
    else
    {
      vd_path.push_back(ladder_rungs_[rung - 1][idx]);
    }
    prev_rung = rung;
  }

  // Populate the result to be returned
  SearchResult<FloatType> result;
  result.trajectory = BGLSolverBase<FloatType>::toStates(vd_path);
  result.cost = cost;

  return result;
}

template <typename FloatType>
SearchResult<FloatType> BGLOMPLRRTSolver<FloatType>::search()
{
  // Initialize the ompl space
  BGLOMPLSolver<FloatType>::initOMPL();

  // Define and setup the RRT planner
  std::shared_ptr<ompl::geometric::RRT> planner =
      std::make_shared<ompl::geometric::RRT>((BGLOMPLSolver<FloatType>::ss_->getSpaceInformation()));
  planner->setRange(BGLOMPLSolver<FloatType>::max_dist_);

  // Get the resulting path through the graph
  SearchResult<FloatType> result = descartes_light::BGLOMPLSolver<FloatType>::ompl_search(planner);

  return result;
}

template <typename FloatType>
SearchResult<FloatType> BGLOMPLRRTConnectSolver<FloatType>::search()
{
  // Initialize the ompl space
  BGLOMPLSolver<FloatType>::initOMPL();

  // Define and setup the RRT planner
  std::shared_ptr<ompl::geometric::RRTConnect> planner =
      std::make_shared<ompl::geometric::RRTConnect>((BGLOMPLSolver<FloatType>::ss_->getSpaceInformation()));
  planner->setRange(BGLOMPLSolver<FloatType>::max_dist_);

  // Get the resulting path through the graph
  SearchResult<FloatType> result = descartes_light::BGLOMPLSolver<FloatType>::ompl_search(planner);

  return result;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_OMPL_SOLVER_HPP
