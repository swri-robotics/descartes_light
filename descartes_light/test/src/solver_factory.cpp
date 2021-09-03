#include <descartes_light/test/impl/solver_factory.hpp>

namespace descartes_light
{
template struct SolverFactory<LadderGraphSolverF>;
template struct SolverFactory<LadderGraphSolverD>;
// Naive BGL Dijkstra search, static vertex static edge
template struct SolverFactory<BGLDijkstraSVSESolver<float, boost::null_visitor>>;
template struct SolverFactory<BGLDijkstraSVSESolver<double, boost::null_visitor>>;
// BGL Dijkstra search with early termination, static vertex static edge
template struct SolverFactory<BGLDijkstraSVSESolverF>;
template struct SolverFactory<BGLDijkstraSVSESolverD>;
// Naive BGL Dijkstra search, static vertex dynamic edge
template struct SolverFactory<BGLDijkstraSVDESolver<float, boost::null_visitor>>;
template struct SolverFactory<BGLDijkstraSVDESolver<double, boost::null_visitor>>;
// BGL Dijkstra searhc with early termination, static vertex dynamic edge
template struct SolverFactory<BGLDijkstraSVDESolverF>;
template struct SolverFactory<BGLDijkstraSVDESolverD>;

}  // namespace descartes_light
