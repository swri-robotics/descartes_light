#include <descartes_light/test/impl/solver_factory.hpp>

namespace descartes_light
{
template struct SolverFactory<LadderGraphSolverF>;
template struct SolverFactory<LadderGraphSolverD>;
template struct SolverFactory<BGLLadderGraphSolverF>;
template struct SolverFactory<BGLLadderGraphSolverD>;
}  // namespace descartes_light
