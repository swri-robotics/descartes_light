#include "descartes_light/edge_sampler.h"

descartes_light::DistanceEdgeEvaluator::DistanceEdgeEvaluator()
{

}

static void considerEdge(const double* start, const double* end, std::size_t next_idx,
                         descartes_light::LadderGraph<double>::EdgeList& out)
{
  const static std::vector<double> vel_limits {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

//  descartes_light::Edge edge;
  double cost = 0.0;
  for (int i = 0; i < 6; ++i)
  {
    double step = end[i] - start[i];
    if (std::abs(step) > vel_limits[i]) return;

    cost += std::pow(step, 2);
  }

  out.push_back({static_cast<float>(cost), next_idx});
}

bool descartes_light::DistanceEdgeEvaluator::evaluate(const Rung_<double>& from, const Rung_<double>& to,
                                                      std::vector<LadderGraph<double>::EdgeList>& edges)
{
  const auto n_start = from.data.size() / 6;
  const auto n_end = to.data.size() / 6;

  // Allocate
  edges.resize(n_start);

  for (std::size_t i = 0; i < n_start; ++i)
  {
    const auto* start_vertex = from.data.data() + 6 * i;
    for (std::size_t j = 0; j < n_end; ++j)
    {
      const auto* end_vertex = to.data.data() + 6 * i;

      // Consider the edge:
      considerEdge(start_vertex, end_vertex, j, edges[i]);
    }
  }

  for (const auto& rung : edges)
    if (!rung.empty())
      return true;

  return false;
}
