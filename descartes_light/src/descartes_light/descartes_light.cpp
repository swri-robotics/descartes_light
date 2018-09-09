#include "descartes_light/descartes_light.h"


descartes_light::Solver::Solver(std::size_t dof)
  : graph_{dof}
{

}

bool descartes_light::Solver::build(const std::vector<descartes_light::PositionSamplerPtr>& trajectory,
                                    EdgeEvaluatorPtr edge_eval)
{
  graph_.resize(trajectory.size());

  std::vector<std::size_t> failed_vertex_samplers;
  std::vector<std::size_t> failed_edge_samplers;

  // Build Vertices
  for (std::size_t i = 0; i < trajectory.size(); ++i)
  {
    std::vector<double> vertex_data;
    if (trajectory[i]->sample(vertex_data))
      graph_.getRung(i).data = std::move(vertex_data);
    else
      failed_vertex_samplers.push_back(i);
  }

  // Build Edges
  for (std::size_t i = 1; i < trajectory.size(); ++i)
  {
    const auto& from = graph_.getRung(i - 1);
    const auto& to = graph_.getRung(i);

    if (!edge_eval->evaluate(from, to, graph_.getEdges(i - 1)))
      failed_edge_samplers.push_back(i-1);
  }

  return failed_edge_samplers.empty() && failed_vertex_samplers.empty();
}

bool descartes_light::Solver::search()
{

}
