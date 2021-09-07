#pragma once

#include <descartes_light/core/solver.h>

namespace descartes_light
{
/**
 * @brief Object for configuring a Descartes solver for the unit test fixture
 */
template <typename SolverT>
struct SolverFactory
{
  typename Solver<typename SolverT::FloatT>::Ptr create(long n_waypoints) const;
};

}  // namespace descartes_light
