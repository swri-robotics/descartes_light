#include "descartes_light/descartes_light.h"
#include "descartes_light/ladder_graph_dag_search.h"
#include <iostream>

static void reportFailedEdges(const std::vector<std::size_t>& indices)
{
  if (indices.empty())
    std::cout << "No failed edges\n";
  else
  {
    std::cout << "Failed edges:\n";
    for (const auto& i : indices)
      std::cout << "\t" << i << "\n";
  }
}


static void reportFailedVertices(const std::vector<std::size_t>& indices)
{
  if (indices.empty())
    std::cout << "No failed vertices\n";
  else
  {
    std::cout << "Failed vertices:\n";
    for (const auto& i : indices)
      std::cout << "\t" << i << "\n";
  }
}

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

  reportFailedVertices(failed_vertex_samplers);
  reportFailedEdges(failed_edge_samplers);

  return failed_edge_samplers.empty() && failed_vertex_samplers.empty();
}

bool descartes_light::Solver::search(std::vector<double>& solution)
{
  DAGSearch s (graph_);
  const auto cost = s.run();

  if (cost == std::numeric_limits<double>::max())
    return false;

  const auto indices = s.shortestPath();

  for (std::size_t i = 0; i < indices.size(); ++i)
  {
    const auto* pose = graph_.vertex(i, indices[i]);
    solution.insert(end(solution), pose, pose + 6);
  }

  std::cout << "Solution found w/ cost = " << cost << "\n";

  return true;
}
