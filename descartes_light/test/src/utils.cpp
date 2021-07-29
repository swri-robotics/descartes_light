#include <descartes_light/test/impl/utils.hpp>

namespace descartes_light
{
template class RandomStateSampler<double>;
template class RandomStateSampler<float>;

template class NaiveEdgeEvaluator<double>;
template class NaiveEdgeEvaluator<float>;

template class NaiveStateEvaluator<double>;
template class NaiveStateEvaluator<float>;

template std::vector<WaypointSampler<double>::ConstPtr>
createSamplers(std::size_t dof, std::size_t n_waypoints, std::size_t samples_per_waypoint, double state_cost);

template std::vector<WaypointSampler<float>::ConstPtr>
createSamplers(std::size_t dof, std::size_t n_waypoints, std::size_t samples_per_waypoint, float state_cost);

}  // namespace descartes_light
