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

  auto mod_ladder_rungs = std::move(ladder_rungs_);
  std::vector<descartes_light::VertexDesc<FloatType>> blank_rung = {source_};
  mod_ladder_rungs.insert(mod_ladder_rungs.begin(), blank_rung);
  mod_ladder_rungs.push_back(blank_rung);


  dss_ = std::make_shared<descartes_light::DescartesStateSpace<FloatType>>(graph_,
                                                                           mod_ladder_rungs,
                                                                           edge_eval_,
                                                                           max_dist_,
                                                                           rung_to_rung_dist_);

  dss_->setLongestValidSegmentFraction(0.5);
  ss_ = std::make_shared<ompl::geometric::SimpleSetup>(dss_);
  ompl::base::ScopedState<descartes_light::DescartesStateSpace<FloatType>> start(dss_), goal(dss_);

  std::pair<long unsigned int, long unsigned int> start_v(0, 0);
  std::pair<long unsigned int, long unsigned int> goal_v(mod_ladder_rungs.size() - 1, 0);
  start->vertex = start_v;
  goal->vertex = goal_v;
  ss_->setStartAndGoalStates(start, goal);
  ss_->setStateValidityChecker([](const ompl::base::State* /*state*/)
  {
    return true;
  });
  ss_->getSpaceInformation()->setMotionValidator(
        std::make_shared<descartes_light::DescartesMotionValidator<FloatType>>(ss_->getSpaceInformation()));

}

template <typename FloatType>
SearchResult<FloatType> BGLOMPLSolver<FloatType>::ompl_search(std::shared_ptr<ompl::base::Planner> ompl_planner)
{
  // Convenience aliases
  const auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;

  ss_->setPlanner(ompl_planner);

  ss_->setup();

  // Perform the search
  ss_->solve(planning_time_);
  ompl::geometric::PathGeometric path = ss_->getSolutionPath();
  std::vector<ompl::base::State *> path_states = path.getStates();
  FloatType cost = 0;
  for (std::size_t i = 0; i < path_states.size() - 1; i++)
  {
    auto curr_state = path_states[i];
    auto next_state = path_states[i + 1];
    double dist = dss_->distance(curr_state, next_state);
    if (dist == dss_->getDistanceEpsilon())
      dist = 0;
    cost += static_cast<FloatType>(dist);
  }

  // Reconstruct the path from the predecesor map; remove the artificial start state
  std::vector<descartes_light::VertexDesc<FloatType>> vd_path;
  std::size_t rung_offset = 0;
  std::size_t prev_rung = 0;
  for (std::size_t i = 0; i < ladder_rungs_.size(); i++)
  {
    std::size_t rung = path_states[i+rung_offset]->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.first;
    std::size_t idx = path_states[i+rung_offset]->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.second;
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

  SearchResult<FloatType> result;

  result.trajectory = BGLSolverBase<FloatType>::toStates(vd_path);
  result.cost = cost;

  return result;
}

template <typename FloatType>
SearchResult<FloatType> BGLOMPLRRTSolver<FloatType>::search()
{
  BGLOMPLSolver<FloatType>::initOMPL();

  std::shared_ptr<ompl::geometric::RRT> planner =
      std::make_shared<ompl::geometric::RRT>((BGLOMPLSolver<FloatType>::ss_->getSpaceInformation()));
  planner->setRange(BGLOMPLSolver<FloatType>::max_dist_);
  SearchResult<FloatType> result = descartes_light::BGLOMPLSolver<FloatType>::ompl_search(planner);

  return result;
}

template <typename FloatType>
SearchResult<FloatType> BGLOMPLRRTConnectSolver<FloatType>::search()
{
  BGLOMPLSolver<FloatType>::initOMPL();

  std::shared_ptr<ompl::geometric::RRTConnect> planner =
      std::make_shared<ompl::geometric::RRTConnect>((BGLOMPLSolver<FloatType>::ss_->getSpaceInformation()));
  planner->setRange(BGLOMPLSolver<FloatType>::max_dist_);
  SearchResult<FloatType> result = descartes_light::BGLOMPLSolver<FloatType>::ompl_search(planner);

  return result;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_OMPL_SOLVER_HPP
