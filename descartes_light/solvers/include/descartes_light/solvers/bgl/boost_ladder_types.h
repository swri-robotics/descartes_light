#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <descartes_light/types.h>
namespace descartes_light
{
template <typename FloatType>
using EdgeProperty = boost::property<boost::edge_weight_t, FloatType>;

template <typename FloatType>
using bglgraph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, StateSample<FloatType>, EdgeProperty<FloatType>>;

template <typename FloatType>
using VertexDesc = typename bglgraph<FloatType>::vertex_descriptor;

template <typename FloatType>
using VertIterator = typename bglgraph<FloatType>::vertex_iterator;

}  // namespace descartes_light
