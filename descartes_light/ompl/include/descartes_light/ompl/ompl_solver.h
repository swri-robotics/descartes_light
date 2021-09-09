#ifndef DESCARTES_LIGHT_SOLVERS_OMPL_OMPL_SOLVER_H
#define DESCARTES_LIGHT_SOLVERS_OMPL_OMPL_SOLVER_H

#include <descartes_light/bgl/bgl_solver.h>
#include <descartes_light/ompl/descartes_space.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>

namespace descartes_light
{
/**
 * @brief BGL solver implementation that constructs vertices and edges in the build function and uses Dijkstra's
 * algorithm with a default visitor to search the graph
 */
template <typename FloatType>
class BGLOMPLSolver : public BGLSolverBaseSVDE<FloatType>
{
public:
  using BGLSolverBaseSVDE<FloatType>::BGLSolverBaseSVDE;
  BGLOMPLSolver(double max_dist,
                double planning_time,
                unsigned num_threads = std::thread::hardware_concurrency(),
                double rung_to_rung_dist = 100000)
    : max_dist_(max_dist)
    , planning_time_(planning_time)
    , BGLSolverBaseSVDE<FloatType>(num_threads)
    , rung_to_rung_dist_(rung_to_rung_dist)
  {
  }

  SearchResult<FloatType> ompl_search(std::shared_ptr<ompl::base::Planner> ompl_planner);
  void initOMPL();

//  SearchResult<FloatType> search() override;

protected:
  double max_dist_;
  double planning_time_;
  double rung_to_rung_dist_;
  std::shared_ptr<descartes_light::DescartesStateSpace<FloatType>> dss_;
  ompl::geometric::SimpleSetupPtr ss_ {nullptr};
};
/**
 * @brief BGL solver implementation that constructs vertices and edges in the build function and uses Dijkstra's
 * algorithm with a default visitor to search the graph
 */
template <typename FloatType>
class BGLOMPLRRTSolver : public BGLOMPLSolver<FloatType>
{
public:
  using BGLOMPLSolver<FloatType>::BGLOMPLSolver;

  SearchResult<FloatType> search() override;
};

using BGLOMPLRRTSolverF = BGLOMPLRRTSolver<float>;
using BGLOMPLRRTSolverD = BGLOMPLRRTSolver<double>;

template <typename FloatType>
class BGLOMPLRRTConnectSolver : public BGLOMPLSolver<FloatType>
{
public:
  using BGLOMPLSolver<FloatType>::BGLOMPLSolver;

  SearchResult<FloatType> search() override;
};

using BGLOMPLRRTConnectSolverF = BGLOMPLRRTConnectSolver<float>;
using BGLOMPLRRTConnectSolverD = BGLOMPLRRTConnectSolver<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_OMPL_OMPL_SOLVER_H
