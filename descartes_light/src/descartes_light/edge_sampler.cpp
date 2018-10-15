#include "descartes_light/edge_sampler.h"
#include <cmath>

descartes_light::DistanceEdgeEvaluator::DistanceEdgeEvaluator(const std::vector<double>& velocity_limits)
  : velocity_limits_(velocity_limits)
{}

static void considerEdge(const double* start, const double* end,
                         const std::vector<double>& delta_thresholds, std::size_t next_idx,
                         descartes_light::LadderGraph<double>::EdgeList& out)
{
  double cost = 0.0;
  for (std::size_t i = 0; i < delta_thresholds.size(); ++i)
  {
    double step = end[i] - start[i];
    if (std::abs(step) > delta_thresholds[i]) return;

    cost += std::pow(step, 2);
  }

  out.emplace_back(cost, next_idx);
}

bool descartes_light::DistanceEdgeEvaluator::evaluate(const Rung_<double>& from, const Rung_<double>& to,
                                                      std::vector<LadderGraph<double>::EdgeList>& edges)
{
  const auto dof = velocity_limits_.size();
  const auto n_start = from.data.size() / dof;
  const auto n_end = to.data.size() / dof;

  // Compute thresholds
  const auto dt = to.timing;

  std::vector<double> delta_thresholds (velocity_limits_.size());
  std::transform(velocity_limits_.begin(), velocity_limits_.end(), delta_thresholds.begin(),
                 [dt] (const double vel_limit) {
    const static double safety_factor = 0.9;
    return dt.upper * vel_limit * safety_factor;
  });

  // Allocate
  edges.resize(n_start);

  for (std::size_t i = 0; i < n_start; ++i)
  {
    const auto* start_vertex = from.data.data() + dof * i;
    for (std::size_t j = 0; j < n_end; ++j)
    {
      const auto* end_vertex = to.data.data() + dof * j;

      // Consider the edge:
      considerEdge(start_vertex, end_vertex, delta_thresholds, j, edges[i]);
    }
  }

  for (const auto& rung : edges)
    if (!rung.empty())
      return true;

  return false;
}
