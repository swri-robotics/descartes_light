#include <gtest/gtest.h>
#include <descartes_light/ladder_graph/ladder_graph_dag_search.h>
#include <descartes_light/ladder_graph/ladder_graph.h>
#include <functional>
#include <iostream>
#include <random>
#include <numeric>
#include <vector>
#include <list>

static std::mt19937 RAND_GEN(0);

template <typename FloatType>
Eigen::Matrix<FloatType, Eigen::Dynamic, 1> generateRandomState(Eigen::Index dof)
{
  static std::normal_distribution<FloatType> dist;
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> sol(dof);
  for (Eigen::Index i = 0; i < dof; ++i)
    sol(i) = dist(RAND_GEN);

  return sol;
}

template <typename FloatType>
void addRandomVertices(descartes_light::LadderGraph<FloatType>& graph, const int max_vertices_per_rung)
{
  // Create a random number generator for determining how many vertices are in each rung
  const int min_vertices_per_rung = std::max(1, max_vertices_per_rung / 2);
  std::uniform_int_distribution<> vertex_count_dist(min_vertices_per_rung, max_vertices_per_rung);

  // Create random rungs
  for (std::size_t rung_idx = 0; rung_idx < graph.size(); ++rung_idx)
  {
    // Create a random number of vertices for this rung
    std::size_t n_vertices = static_cast<std::size_t>(vertex_count_dist(RAND_GEN));

    // Create random joint states for each vertex
    std::vector<Eigen::VectorXd> sols(n_vertices);
    auto& rung = graph.getRung(rung_idx);
    for (std::size_t i = 0; i < n_vertices; ++i)
      rung.nodes.push_back(
          descartes_light::Node<FloatType>(generateRandomState<FloatType>(static_cast<Eigen::Index>(graph.dof())), 0));
  }
}

template <typename FloatType>
void addRandomEdges(descartes_light::LadderGraph<FloatType>& graph)
{
  // Create random edges
  for (std::size_t rung_idx = 0; rung_idx < graph.size() - 1; ++rung_idx)
  {
    // Choose a random subset of vertices in the next rung to connect to
    const std::size_t next_rung_size = graph.rungSize(rung_idx + 1);
    std::uniform_int_distribution<std::size_t> edge_count_dist(1, next_rung_size);
    std::normal_distribution<FloatType> cost_dist(10.0);

    auto& rung = graph.getRung(rung_idx);
    std::vector<typename descartes_light::LadderGraph<FloatType>::EdgeList> edges_list;
    for (std::size_t vertex_idx = 0; vertex_idx < rung.nodes.size(); ++vertex_idx)
    {
      auto& node = rung.nodes[vertex_idx];

      // Number of connections for this vertex
      const std::size_t n_edges = edge_count_dist(RAND_GEN);

      // Create a randomly shuffled list of vertex indices from the next rung
      std::vector<unsigned> next_rung_vertex_idx(next_rung_size);
      std::iota(next_rung_vertex_idx.begin(), next_rung_vertex_idx.end(), 0);
      std::random_shuffle(next_rung_vertex_idx.begin(), next_rung_vertex_idx.end());

      // Create the edges for this vertex
      node.edges.resize(n_edges);
      for (std::size_t i = 0; i < n_edges; ++i)
      {
        auto& edge = node.edges[i];
        edge.cost = cost_dist(RAND_GEN);
        edge.idx = next_rung_vertex_idx[i];
      }
    }
  }
}

template <typename FloatType>
descartes_light::LadderGraph<FloatType> generateRandomGraph(const std::size_t dof,
                                                            const std::size_t n_rungs,
                                                            const int max_vertices_per_rung)
{
  descartes_light::LadderGraph<FloatType> graph(dof);
  graph.resize(n_rungs);

  addRandomVertices(graph, max_vertices_per_rung);
  addRandomEdges(graph);

  return graph;
}

template <typename FloatType>
void runNoEdgesTest()
{
  descartes_light::LadderGraph<FloatType> graph(6);
  graph.resize(10);
  addRandomVertices<FloatType>(graph, 10);

  descartes_light::DAGSearch<FloatType> search(graph);

  FloatType total_cost = search.run();
  ASSERT_NEAR(total_cost, std::numeric_limits<FloatType>::max(), 1.0e-6);
}

template <typename FloatType>
void runCorrectGraphTest()
{
  const std::size_t dof = 6;
  const std::size_t n_rungs = 10;
  const int max_vertices_per_rung = 10;

  descartes_light::LadderGraph<FloatType> graph = generateRandomGraph<FloatType>(dof, n_rungs, max_vertices_per_rung);
  descartes_light::DAGSearch<FloatType> search(graph);
  FloatType total_cost;
  ASSERT_NO_THROW(total_cost = search.run());
  EXPECT_GT(total_cost, 0.0);

  std::cout << "Total cost: " << total_cost << std::endl;

  std::vector<typename descartes_light::DAGSearch<FloatType>::predecessor_t> path;
  ASSERT_NO_THROW(path = search.shortestPath());
  EXPECT_EQ(path.size(), n_rungs);

  std::cout << "Path:\n";
  // Reconstruct the path cost
  FloatType reconstructed_cost = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    const auto vertex_idx = path[i];
    std::cout << vertex_idx << ", ";

    if (i < path.size() - 1)
    {
      const auto next_idx = path[i + 1];
      const auto& edge_list = graph.getRung(i).nodes[vertex_idx].edges;
      auto it = std::find_if(edge_list.begin(),
                             edge_list.end(),
                             [next_idx](const descartes_light::Edge<FloatType>& edge) { return edge.idx == next_idx; });

      reconstructed_cost += it->cost;
    }
  }
  std::cout << std::endl;

  ASSERT_TRUE(std::abs(total_cost - reconstructed_cost) < std::numeric_limits<FloatType>::epsilon());
}

template <typename FloatType>
void runKnownPathTest()
{
  const std::size_t dof = 6;
  const std::size_t n_rungs = 10;
  const int max_vertices_per_rung = 10;

  descartes_light::LadderGraph<FloatType> graph = generateRandomGraph<FloatType>(dof, n_rungs, max_vertices_per_rung);

  std::vector<std::size_t> known_path;
  known_path.reserve(graph.size());

  // Choose a random start vertex and add it to the path
  {
    std::uniform_int_distribution<std::size_t> vertex_idx_dist(0, graph.getRung(0).nodes.size() - 1);
    std::size_t vertex_idx = vertex_idx_dist(RAND_GEN);
    known_path.push_back(vertex_idx);
  }

  for (std::size_t i = 0; i < graph.size() - 1; ++i)
  {
    // Get the current rung
    auto& rung = graph.getRung(i);

    // Choose a random edge from the selected vertex and set its cost to 0 so the search should choose it
    auto& edges = rung.nodes[known_path[i]].edges;
    std::uniform_int_distribution<std::size_t> edge_idx_dist(0, edges.size() - 1);
    std::size_t edge_idx = edge_idx_dist(RAND_GEN);
    auto& edge = edges[edge_idx];
    edge.cost = 0.0;

    // Add the vertex associated with this edge to the path
    known_path.push_back(edge.idx);
  }

  descartes_light::DAGSearch<FloatType> search(graph);
  FloatType total_cost = search.run();
  EXPECT_NEAR(total_cost, 0.0, std::numeric_limits<FloatType>::epsilon());

  auto path = search.shortestPath();
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    EXPECT_EQ(path.at(i), known_path.at(i));
  }
}

TEST(DAGSearch, NoEdges)
{
  runNoEdgesTest<double>();
  runNoEdgesTest<float>();
}

TEST(DAGSearch, CorrectGraph)
{
  runCorrectGraphTest<double>();
  runCorrectGraphTest<float>();
}

TEST(DAGSearch, KnownPathTest)
{
  runKnownPathTest<double>();
  runKnownPathTest<float>();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
