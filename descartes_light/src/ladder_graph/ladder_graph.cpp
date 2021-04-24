#include <descartes_light/impl/ladder_graph.hpp>
#include <vector>
#include <list>

namespace descartes_light
{
// Explicit template specialization
template class LadderGraph<float, std::vector>;
template class LadderGraph<double, std::vector>;
template class LadderGraph<float, std::list>;
template class LadderGraph<double, std::list>;

}  // namespace descartes_light
