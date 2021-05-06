#include <descartes_light/solvers/ladder_graph/impl/ladder_graph.hpp>
#include <vector>
#include <list>

namespace descartes_light
{
// Explicit template specialization
template class LadderGraph<float>;
template class LadderGraph<double>;

}  // namespace descartes_light
