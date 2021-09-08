#ifndef DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_OMPL_SOLVER_HPP
#define DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_OMPL_SOLVER_HPP

#include <descartes_light/ompl/ompl_solver.h>
#include <descartes_light/bgl/impl/event_visitors.hpp>
#include <descartes_light/ompl/descartes_space.h>

#include <descartes_light/descartes_macros.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>

namespace descartes_light
{
template <typename FloatType>
SearchResult<FloatType> BGLOMPLSVDESolver<FloatType>::search()
{
  // Convenience aliases
  auto& graph_ = BGLSolverBase<FloatType>::graph_;
  const auto& source_ = BGLSolverBase<FloatType>::source_;
//  auto& predecessor_map_ = BGLSolverBase<FloatType>::predecessor_map_;
  const auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;
  const auto& edge_eval_ = BGLSolverBaseSVDE<FloatType>::edge_eval_;
  std::cout << "EE size: " << edge_eval_.size() << std::endl;
  double max_dist = 0.5;

  // Internal properties
//  auto index_prop_map = boost::get(boost::vertex_index, graph_);
//  auto weight_prop_map = boost::get(boost::edge_weight, graph_);
//  auto color_prop_map = boost::get(&Vertex<FloatType>::color, graph_);
//  auto distance_prop_map = boost::get(&Vertex<FloatType>::distance, graph_);

//  predecessor_map_.clear();
//  boost::associative_property_map<std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>> predecessor_prop_map(
//      predecessor_map_);

  ompl::base::StateSpacePtr state_space_ptr;

  auto dss = std::make_shared<descartes_light::DescartesStateSpace<FloatType>>(graph_, ladder_rungs_, edge_eval_, max_dist);
  std::cout << "1" << std::endl;

  auto rvss = std::make_shared<ompl::base::RealVectorStateSpace>(2);
  auto dscss = std::make_shared<ompl::base::DiscreteStateSpace>(0,39);

  std::function<ompl::base::StateSamplerPtr(const descartes_light::DescartesStateSpace<FloatType> *)> descartes_sampler;
  std::cout << "2" << std::endl;
//  auto sampler = dss->allocDefaultStateSampler();
//  auto sampler = rvss->allocDefaultStateSampler();
//  auto sampler = dscss->allocDefaultStateSampler();
  std::cout << "3" << std::endl;

//  dss->setLongestValidSegmentFraction(1 / static_cast<double>(ladder_rungs_.size()));
  dss->setLongestValidSegmentFraction(0.5);
  ompl::geometric::SimpleSetup ss(dss);
//  ompl::geometric::SimpleSetup ss(rvss);
//  ompl::geometric::SimpleSetup ss(dscss);
  std::cout << "4" << std::endl;

  ompl::base::ScopedState<descartes_light::DescartesStateSpace<FloatType>> start(dss), goal(dss);
//  ompl::base::ScopedState<> start(dss), goal(dss);
//  ompl::base::ScopedState<> start(rvss), goal(rvss);
  std::cout << "5" << std::endl;

  unsigned int i = 0;
//  descartes_light::DescartesStateSpace<FloatType>::StateType start_state;
//  start_state.value = 1;
//  ompl::base::ScopedState<ompl::base::DiscreteStateSpace> start(dscss), goal(dscss);
  std::cout << "6" << std::endl;
  std::cout << "ladder_rungs_.front().front()" << ladder_rungs_.front().front() << std::endl;
  std::cout << "ladder_rungs_.back().back()" << ladder_rungs_.back().back() << std::endl;
  std::pair<long unsigned int, long unsigned int> start_v(0, 2);
//  std::pair<long unsigned int, long unsigned int> goal_v(ladder_rungs_.size() - 1, ladder_rungs_.back().size() - 1);
  std::pair<long unsigned int, long unsigned int> goal_v(9, 2);
//  start->value = ladder_rungs_.front().front();
//  goal->value = ladder_rungs_.back().back();
  start->vertex = start_v;
  goal->vertex = goal_v;
  std::cout << "start " << start << std::endl;
//  std::cout << "start_state" << start_state.value << std::endl;
  std::cout << "goal " << goal << std::endl;
  std::cout << "7" << std::endl;

//  std::shared_ptr<descartes_light::DescartesStateSampler<FloatType>> sampler(dss);
  auto sampler = dss->allocStateSampler();
  std::cout << "8" << std::endl;
//  descartes_light::DescartesStateSampler smplr(dss);
  ompl::base::State *ref_state = dss->allocState();
  ompl::base::State *test_state = dss->allocState();
  std::cout << "9" << std::endl;
  for (std::size_t i = 0; i < 10; i++)
  {
    sampler->sampleUniform(ref_state);
    sampler->sampleUniform(test_state);
    std::cout << "ref_state " << i << ": ";
    dss->printState(ref_state, std::cout);
    std::cout << "test_state " << i << ": ";
    dss->printState(test_state, std::cout);
    std::cout << "distance: " << dss->distance(ref_state, test_state) << std::endl;
  }
//  for (std::size_t i = 0; i < ladder_rungs_.size() - 1; i++)
//  {
//    for (std::size_t j = 0; j < 4; j++)
//    {
//      for (std::size_t k = 0; k < 4; k++)
//      {
//        std::pair<long unsigned int, long unsigned int> vertex1(i, j);
//        ref_state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex = vertex1;
//        std::pair<long unsigned int, long unsigned int> vertex2(i + 1, k);
//        test_state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex = vertex2;
//        std::cout << "ref_state " << i << ": ";
//        dss->printState(ref_state, std::cout);
//        std::cout << "test_state " << i << ": ";
//        dss->printState(test_state, std::cout);
//        std::cout << "distance: " << dss->distance(ref_state, test_state) << std::endl;;
//      }
//    }
//  }
  std::cout << "9.1" << std::endl;
//  auto si = ss.getSpaceInformation();
  ompl::base::SpaceInformationPtr si(ss.getSpaceInformation());
  auto mv = std::make_shared<descartes_light::DescartesMotionValidator<FloatType>>(si);
  ss.getSpaceInformation()->setMotionValidator(mv);
//  ss.getSpaceInformation()->setMotionValidator(
//        std::make_shared<descartes_light::DescartesMotionValidator<FloatType>(si));

  std::cout << "10" << std::endl;
//  std::shared_ptr<ompl::geometric::RRTConnect> planner =
//      std::make_shared<ompl::geometric::RRTConnect>((ss.getSpaceInformation()));
  std::shared_ptr<ompl::geometric::RRT> planner =
      std::make_shared<ompl::geometric::RRT>((ss.getSpaceInformation()));
  std::cout << "10.1" << std::endl;
//  planner->setRange(1000.0);
  planner->setRange(max_dist);
  std::cout << "10.2" << std::endl;
  ss.setPlanner(planner);
  std::cout << "10.3" << std::endl;

  ss.setStartAndGoalStates(start, goal);
  std::cout << "11" << std::endl;

  ss.setStateValidityChecker([](const ompl::base::State *state)
  {
//    std::size_t rung =
//        state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.first;
//    std::size_t idx =
//        state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.second;
//    if (rung == 5 && (idx == 0 || idx ==1 || idx ==2))
//    {
////      std::cout << "RUNG: " << rung << " IDX: " << idx << std::endl;
//      return false;
//    }
//    if (rung ==5)
//    {
//      std::cout << "RUNG: " << rung << " IDX: " << idx << std::endl;
//    }
//    if (rung == 6 && idx==3)
//      return false;
    return true;
  });
  std::cout << "11.5" << std::endl;
  ss.print();
  std::cout << "11.75" << std::endl;

  ss.setup();
  std::cout << "12" << std::endl;
  ss.print();
  std::cout << "13" << std::endl;
  ss.solve(30.0);
  std::cout << "14" << std::endl;
  ompl::geometric::PathGeometric path = ss.getSolutionPath();
  std::cout << "15" << std::endl;
  path.print(std::cout);
  std::cout << "16" << std::endl;
  std::vector<ompl::base::State *> path_states = path.getStates();
  for (std::size_t i = 0; i < path_states.size() - 1; i++)
  {
    auto curr_state = path_states[i];
    auto next_state = path_states[i + 1];
    std::cout << "FROM: ";
    dss->printState(curr_state, std::cout);
    std::cout << "TO: ";
    dss->printState(next_state, std::cout);
    std::cout << "Distance: " << dss->distance(curr_state, next_state) << std::endl << std::endl;
  }
//  ss.simplifySolution();
//  std::cout << "17" << std::endl;
//  ompl::geometric::PathGeometric path2 = ss.getSolutionPath();
//  std::cout << "18" << std::endl;
//  path2.print(std::cout);
//  std::cout << "19" << std::endl;


  dss->printSettings(std::cout);

  // Perform the search
//  boost::dijkstra_shortest_paths(graph_,
//                                 source_,
//                                 predecessor_prop_map,
//                                 distance_prop_map,
//                                 weight_prop_map,
//                                 index_prop_map,
//                                 std::less<>(),
//                                 std::plus<>(),
//                                 std::numeric_limits<FloatType>::max(),
//                                 static_cast<FloatType>(0.0),
//                                 boost::default_dijkstra_visitor(),
//                                 color_prop_map);

//  // Find lowest cost node in last rung
//  auto target = std::min_element(ladder_rungs_.back().begin(),
//                                 ladder_rungs_.back().end(),
//                                 [&](const VertexDesc<FloatType>& a, const VertexDesc<FloatType>& b) {
//                                   return graph_[a].distance < graph_[b].distance;
//                                 });

  SearchResult<FloatType> result;

  // Reconstruct the path from the predecesor map; remove the artificial start state
//  const auto vd_path = BGLSolverBase<FloatType>::reconstructPath(source_, *target);
//  result.trajectory = BGLSolverBase<FloatType>::toStates(vd_path);
//  result.trajectory.erase(result.trajectory.begin());

//  result.cost = graph_[*target].distance;

  return result;
}

template <typename FloatType>
SearchResult<FloatType> BGLEfficientOMPLSVSESolver<FloatType>::search()
{
  // Convenience aliases
//  auto& graph_ = BGLSolverBase<FloatType>::graph_;
//  const auto& source_ = BGLSolverBase<FloatType>::source_;
//  auto& predecessor_map_ = BGLSolverBase<FloatType>::predecessor_map_;
//  const auto& ladder_rungs_ = BGLSolverBase<FloatType>::ladder_rungs_;

//  // Internal properties
//  auto index_prop_map = boost::get(boost::vertex_index, graph_);
//  auto weight_prop_map = boost::get(boost::edge_weight, graph_);
//  auto color_prop_map = boost::get(&Vertex<FloatType>::color, graph_);
//  auto distance_prop_map = boost::get(&Vertex<FloatType>::distance, graph_);

//  predecessor_map_.clear();
//  boost::associative_property_map<std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>> predecessor_prop_map(
//      predecessor_map_);

//  const long last_rung_idx = static_cast<long>(ladder_rungs_.size() - 1);
//  auto visitor = boost::make_dijkstra_visitor(early_terminator<FloatType>(last_rung_idx));

//  // Perform the search
//  try
//  {
//    boost::dijkstra_shortest_paths(graph_,
//                                   source_,
//                                   predecessor_prop_map,
//                                   distance_prop_map,
//                                   weight_prop_map,
//                                   index_prop_map,
//                                   std::less<>(),
//                                   std::plus<>(),
//                                   std::numeric_limits<FloatType>::max(),
//                                   static_cast<FloatType>(0.0),
//                                   visitor,
//                                   color_prop_map);
//  }
//  catch (const VertexDesc<FloatType>& target)
//  {
//    SearchResult<FloatType> result;

//    // Reconstruct the path from the predecesor map; remove the artificial start state
//    const auto vd_path = BGLSolverBase<FloatType>::reconstructPath(source_, target);
//    result.trajectory = BGLSolverBase<FloatType>::toStates(vd_path);
//    result.trajectory.erase(result.trajectory.begin());

//    result.cost = graph_[target].distance;

//    return result;
//  }

  // If the visitor never threw the vertex descriptor, there was an issue with the search
  throw std::runtime_error("Search failed to encounter vertex associated with the last waypoint in the trajectory");
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_OMPL_SOLVER_HPP
