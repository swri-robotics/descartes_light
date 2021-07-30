#include <descartes_light/test/impl/solver_factory.hpp>

namespace descartes_light
{
template struct SolverFactory<LadderGraphSolverF>;
template struct SolverFactory<LadderGraphSolverD>;
template struct SolverFactory<BGLDijkstraSolverVEF>;
template struct SolverFactory<BGLDijkstraSolverVED>;
template struct SolverFactory<BGLEfficientDijkstraSolverVEF>;
template struct SolverFactory<BGLEfficientDijkstraSolverVED>;
}  // namespace descartes_light
