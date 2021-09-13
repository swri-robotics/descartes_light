#ifndef DESCARTES_LIGHT_SOLVERS_OMPL_OMPL_SOLVER_H
#define DESCARTES_LIGHT_SOLVERS_OMPL_OMPL_SOLVER_H

#include <descartes_light/bgl/bgl_solver.h>
#include <descartes_light/ompl/descartes_space.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>

namespace descartes_light
{
/**
 * @brief BGL solver implementation that constructs vertices in the build function and uses an ompl planner
 * to find a valid path through the ladder graph given a maximum allowable cost and set allowed planning time
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
    : BGLSolverBaseSVDE<FloatType>(num_threads)
    , max_dist_(max_dist)
    , planning_time_(planning_time)
    , rung_to_rung_dist_(rung_to_rung_dist)
  {
  }

  /** @brief A search using an ompl planner */
  SearchResult<FloatType> ompl_search(std::shared_ptr<ompl::base::Planner> ompl_planner);

  /** @brief Method to setup OMPL problem to be called after ladder graph is constructed */
  void initOMPL();

protected:
  /** @brief The maximum allowed cost for an edge connection */
  double max_dist_;

  /** @brief Allowed planning time for algorithm to find a solution, will return before this if a solution is found */
  double planning_time_;

  /** @brief Cost associated with moving an extra rung. Should be comparable to the maximum cost of a connection between adjacent rungs */
  double rung_to_rung_dist_;

  /** @brief Descartes State Space, inherits from base state space in OMPL, created to give ompl a space to search */
  std::shared_ptr<descartes_light::DescartesStateSpace<FloatType>> dss_;

  /** @brief Used for setting parameters for ompl planner and calling solve */
  ompl::geometric::SimpleSetupPtr ss_ {nullptr};
};

/**
 * @brief OMPL solver implementation using RRT
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

/**
 * @brief OMPL solver implementation using RRT Connect
 */
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
