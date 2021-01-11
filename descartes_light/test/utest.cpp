#include <gtest/gtest.h>
#include <descartes_light/ladder_graph_dag_search.h>
#include <descartes_light/ladder_graph.h>
#include <functional>
#include <iostream>
#include <random>

extern template class descartes_light::LadderGraph<double>;
extern template class descartes_light::DAGSearch<double>;

static std::mt19937 RAND_GEN(0);

std::vector<double> generateRandomSolution(const std::size_t dof)
{
  static std::normal_distribution<double> dist;
  std::vector<double> sol(dof);
  std::generate(sol.begin(), sol.end(), []() -> double { return dist(RAND_GEN); });
  return sol;
}

void addRandomVertices(descartes_light::LadderGraphD& graph, const int max_vertices_per_rung)
{
  // Create a random number generator for determining how many vertices are in each rung
  const int min_vertices_per_rung = std::max(1, max_vertices_per_rung / 2);
  std::uniform_int_distribution<> vertex_count_dist(min_vertices_per_rung, max_vertices_per_rung);

  // Create random rungs
  for (std::size_t rung_idx = 0; rung_idx < graph.size(); ++rung_idx)
  {
    // Create a random number of vertices for this rung
    std::size_t n_vertices = static_cast<std::size_t>(vertex_count_dist(RAND_GEN));

    // Create solutions for each vertex
    std::vector<std::vector<double>> sols(n_vertices);
    std::generate(sols.begin(), sols.end(), std::bind(generateRandomSolution, graph.dof()));
    graph.assignRung(rung_idx, {}, sols);
  }
}

void addRandomEdges(descartes_light::LadderGraphD& graph)
{
  // Create random edges
  for (std::size_t rung_idx = 0; rung_idx < graph.size() - 1; ++rung_idx)
  {
    // Choose a random subset of vertices in the next rung to connect to
    const std::size_t next_rung_size = graph.rungSize(rung_idx + 1);
    std::uniform_int_distribution<std::size_t> edge_count_dist(1, next_rung_size);
    std::normal_distribution<double> cost_dist(10.0);

    std::vector<descartes_light::LadderGraphD::EdgeList> edges_list;
    for (std::size_t vertex_idx = 0; vertex_idx < graph.rungSize(rung_idx); ++vertex_idx)
    {
      // Number of connections for this vertex
      const std::size_t n_edges = edge_count_dist(RAND_GEN);

      // Create a randomly shuffled list of vertex indices from the next rung
      std::vector<unsigned> next_rung_vertex_idx(next_rung_size);
      std::iota(next_rung_vertex_idx.begin(), next_rung_vertex_idx.end(), 0);
      std::random_shuffle(next_rung_vertex_idx.begin(), next_rung_vertex_idx.end() /*, RAND_GEN*/);

      // Create the edges for this vertex
      descartes_light::LadderGraphD::EdgeList edges(n_edges);
      for (std::size_t i = 0; i < n_edges; ++i)
      {
        auto& edge = edges[i];
        edge.cost = cost_dist(RAND_GEN);
        edge.idx = next_rung_vertex_idx[i];
      }

      // Add the edge list for this vertex to the list of edge lists for this rung
      edges_list.push_back(edges);
    }

    // Add the edges for all vertices in this rung to the graph
    graph.assignEdges(rung_idx, std::move(edges_list));
  }
}

descartes_light::LadderGraphD generateRandomGraph(const std::size_t dof,
                                                  const std::size_t n_rungs,
                                                  const int max_vertices_per_rung)
{
  descartes_light::LadderGraphD graph(dof);
  graph.resize(n_rungs);

  addRandomVertices(graph, max_vertices_per_rung);
  addRandomEdges(graph);

  return graph;
}

TEST(DAGSearch, NoEdges)
{
  descartes_light::LadderGraphD graph(6);
  graph.resize(10);
  addRandomVertices(graph, 10);

  descartes_light::DAGSearchD search(graph);

  double total_cost = search.run();
  ASSERT_NEAR(total_cost, std::numeric_limits<double>::max(), 1.0e-6);
}

TEST(DAGSearch, CorrectGraph)
{
  const std::size_t dof = 6;
  const std::size_t n_rungs = 10;
  const int max_vertices_per_rung = 10;

  descartes_light::LadderGraphD graph = generateRandomGraph(dof, n_rungs, max_vertices_per_rung);
  descartes_light::DAGSearchD search(graph);
  double total_cost;
  ASSERT_NO_THROW(total_cost = search.run());
  EXPECT_GT(total_cost, 0.0);

  std::cout << "Total cost: " << total_cost << std::endl;

  std::vector<descartes_light::DAGSearchD::predecessor_t> path;
  ASSERT_NO_THROW(path = search.shortestPath());
  EXPECT_EQ(path.size(), n_rungs);

  std::cout << "Path:\n";
  // Reconstruct the path cost
  double reconstructed_cost = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    const auto vertex_idx = path[i];
    std::cout << vertex_idx << ", ";

    if (i < path.size() - 1)
    {
      const auto next_idx = path[i + 1];
      const auto& edge_list = graph.getRung(i).edges[vertex_idx];
      auto it = std::find_if(
          edge_list.begin(), edge_list.end(), [next_idx](const descartes_light::LadderGraphD::Rung::Edge& edge) {
            return edge.idx == next_idx;
          });

      reconstructed_cost += it->cost;
    }
  }
  std::cout << std::endl;

  ASSERT_TRUE(std::abs(total_cost - reconstructed_cost) < std::numeric_limits<double>::epsilon());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
