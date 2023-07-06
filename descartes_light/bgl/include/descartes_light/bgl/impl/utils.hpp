#pragma once

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <fstream>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/bgl/utils.h>

namespace descartes_light
{
template <typename FloatType>
SubGraph<FloatType> createDecoratedSubGraph(const BGLGraph<FloatType>& g)
{
  SubGraph<FloatType> main;
  boost::get_property(main, boost::graph_name) = "G";

  std::map<long, SubGraph<FloatType>*> rung_subgraph_map;

  // Convert colors from enum to strings
  VertexIt<FloatType> start, end;
  boost::tie(start, end) = boost::vertices(g);
  for (auto it = start; it != end; ++it)
  {
    const long rung_idx = g[*it].rung_idx;
    if (rung_subgraph_map.find(rung_idx) == rung_subgraph_map.end())
    {
      // Create a new sub-graph for the rung
      SubGraph<FloatType>& rung_subgraph = main.create_subgraph();
      std::stringstream ss;
      ss << "cluster_" << rung_idx;
      boost::get_property(rung_subgraph, boost::graph_name) = ss.str();

      rung_subgraph_map.emplace(rung_idx, &rung_subgraph);
    }
    SubGraph<FloatType>* rung_subgraph = rung_subgraph_map.at(rung_idx);

    // Add a vertex to this new subgraph
    auto v = boost::add_vertex(*rung_subgraph);

    std::stringstream name;
    name << std::setprecision(4) << "v" << *it << ": " << g[*it].distance;
    boost::get(boost::vertex_attribute, *rung_subgraph)[v][LABEL_ATTR] = name.str();

    // Add colors
    auto& color_prop = boost::get(boost::vertex_attribute, *rung_subgraph)[v][FILLCOLOR_ATTR];
    switch (g[*it].color)
    {
      case boost::default_color_type::white_color:
        color_prop = "white";
        break;
      case boost::default_color_type::gray_color:
        color_prop = "gray";
        break;
      case boost::default_color_type::black_color:
        color_prop = "gray30";
        break;
      case boost::default_color_type::red_color:
        color_prop = "red";
        break;
      default:
        break;
    }
  }

  // Add the edges to the subgraph
  auto weights = boost::get(boost::edge_weight, g);

  EdgeIt<FloatType> first, last;
  boost::tie(first, last) = boost::edges(g);
  for (auto it = first; it != last; ++it)
  {
    VertexDesc<FloatType> source = boost::source(*it, g);
    VertexDesc<FloatType> target = boost::target(*it, g);

    bool added;
    EdgeDesc<FloatType> e;
    boost::tie(e, added) = boost::add_edge(source, target, weights[*it], main);

    std::stringstream ss;
    ss << std::setprecision(4) << weights[*it];
    boost::get(boost::edge_attribute, main)[e][LABEL_ATTR] = ss.str();
  }

  // Set th graph properties
  boost::get_property(main, boost::graph_vertex_attribute)[STYLE_ATTR] = "filled";
  boost::get_property(main, boost::graph_graph_attribute)[RANKDIR_ATTR] = "LR";

  return main;
}

template <typename FloatType>
void writeGraph(const std::string& filename, const BGLGraph<FloatType>& graph)
{
  std::ofstream file(filename);
  if (!file.good())
    throw std::runtime_error("Failed to open file '" + filename + "'");

  boost::write_graphviz(file, createDecoratedSubGraph(graph));
}

}  // namespace descartes_light
