#include <descartes_light/test/impl/solver_factory.hpp>

namespace descartes_light
{
template struct SolverFactory<LadderGraphSolverF>;
template struct SolverFactory<LadderGraphSolverD>;
template struct SolverFactory<BGLDijkstraSVSESolverF>;
template struct SolverFactory<BGLDijkstraSVSESolverD>;
template struct SolverFactory<BGLEfficientDijkstraSVSESolverF>;
template struct SolverFactory<BGLEfficientDijkstraSVSESolverD>;
template struct SolverFactory<DFSAddAllSolverF>;
template struct SolverFactory<DFSAddAllSolverD>;
template struct SolverFactory<DFSSortLadderGraphSolverF>;
template struct SolverFactory<DFSSortLadderGraphSolverD>;
template struct SolverFactory<DFSRandomLadderGraphSolverF>;
template struct SolverFactory<DFSRandomLadderGraphSolverD>;

}  // namespace descartes_light
