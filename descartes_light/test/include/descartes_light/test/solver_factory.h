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
  using FloatType = typename SolverT::FloatT;
  typename Solver<FloatType>::Ptr create() const;
};

}  // namespace descartes_light
