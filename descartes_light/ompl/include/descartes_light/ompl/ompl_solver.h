#ifndef DESCARTES_LIGHT_SOLVERS_OMPL_OMPL_SOLVER_H
#define DESCARTES_LIGHT_SOLVERS_OMPL_OMPL_SOLVER_H

#include <descartes_light/ompl/descartes_space.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph.h>
#include <descartes_light/core/solver.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <thread>
#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
/**
 * @brief Ladder Graph solver implementation that constructs vertices in the build function and uses an ompl planner
 * to find a valid path through the ladder graph given a maximum allowable cost and set allowed planning time
 */
template <typename FloatType>
class LadderGraphOMPLSolver : public Solver<FloatType>
{
public:
  LadderGraphOMPLSolver(double max_dist,
                        double planning_time,
                        unsigned num_threads = std::thread::hardware_concurrency())
    : max_dist_(max_dist), planning_time_(planning_time), rung_to_rung_dist_(max_dist * 1000), num_threads_(num_threads)
  {
  }

  BuildStatus buildImpl(const std::vector<typename WaypointSampler<FloatType>::ConstPtr>& trajectory,
                        const std::vector<typename EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
                        const std::vector<typename StateEvaluator<FloatType>::ConstPtr>& state_eval) override;

  /** @brief A search using an ompl planner */
  SearchResult<FloatType> ompl_search(std::shared_ptr<ompl::base::Planner> ompl_planner);

protected:
  LadderGraph<FloatType> graph_;

  /** @brief The maximum allowed cost for an edge connection */
  double max_dist_;

  /** @brief Allowed planning time for algorithm to find a solution, will return before this if a solution is found */
  double planning_time_;

  /** @brief Cost associated with moving an extra rung. Should be comparable to the maximum cost of a connection between
   * adjacent rungs */
  double rung_to_rung_dist_;

  /** @brief Descartes State Space, inherits from base state space in OMPL, created to give ompl a space to search */
  std::shared_ptr<descartes_light::DescartesStateSpace<FloatType>> dss_;

  /** @brief Used for setting parameters for ompl planner and calling solve */
  ompl::geometric::SimpleSetupPtr ss_{ nullptr };

  /** @brief Number of threads for parallel processing */
  unsigned num_threads_;
};

/**
 * @brief OMPL solver implementation using RRT
 */
template <typename FloatType>
class LadderGraphOMPLRRTSolver : public LadderGraphOMPLSolver<FloatType>
{
public:
  using LadderGraphOMPLSolver<FloatType>::LadderGraphOMPLSolver;

  SearchResult<FloatType> search() override;
};

using LadderGraphOMPLRRTSolverF = LadderGraphOMPLRRTSolver<float>;
using LadderGraphOMPLRRTSolverD = LadderGraphOMPLRRTSolver<double>;

/**
 * @brief OMPL solver implementation using RRT Connect
 */
template <typename FloatType>
class LadderGraphOMPLRRTConnectSolver : public LadderGraphOMPLSolver<FloatType>
{
public:
  using LadderGraphOMPLSolver<FloatType>::LadderGraphOMPLSolver;

  SearchResult<FloatType> search() override;
};

using LadderGraphOMPLRRTConnectSolverF = LadderGraphOMPLRRTConnectSolver<float>;
using LadderGraphOMPLRRTConnectSolverD = LadderGraphOMPLRRTConnectSolver<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_OMPL_OMPL_SOLVER_H
