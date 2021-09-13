#pragma once

#include <descartes_light/bgl/boost_graph_types.h>

namespace descartes_light
{
// Graphviz attribute names
const static std::string FILLCOLOR_ATTR = "fillcolor";
const static std::string STYLE_ATTR = "style";
const static std::string LABEL_ATTR = "label";
const static std::string RANKDIR_ATTR = "rankdir";

/**
 * @brief Creates a sub-graph where the vertices in each conceptual rung of the planning graph are grouped together and
 * assigns properties to the vertices and edges for display in a Graphviz file
 */
template <typename FloatType>
SubGraph<FloatType> createDecoratedSubGraph(const BGLGraph<FloatType>& g);

/**
 * @brief Converts a graph into a decorated sub-graph and writes the sub-graph to a Graphviz file
 */
template <typename FloatType>
void writeGraph(const std::string& filename, const BGLGraph<FloatType>& graph);

}  // namespace descartes_light
