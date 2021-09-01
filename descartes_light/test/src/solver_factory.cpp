#include <descartes_light/test/impl/solver_factory.hpp>

namespace descartes_light
{
template struct SolverFactory<LadderGraphSolverF>;
template struct SolverFactory<LadderGraphSolverD>;
template struct SolverFactory<BGLDijkstraSVSESolverF>;
template struct SolverFactory<BGLDijkstraSVSESolverD>;
template struct SolverFactory<BGLEfficientDijkstraSVSESolverF>;
template struct SolverFactory<BGLEfficientDijkstraSVSESolverD>;
template struct SolverFactory<DepthFirstSVDESolverF>;
template struct SolverFactory<DepthFirstSVDESolverD>;

}  // namespace descartes_light
