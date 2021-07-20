#ifndef DESCARTES_LIGHT_BOOST_VISITORS
#define DESCARTES_LIGHT_BOOST_VISITORS

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <descartes_light/solvers/bgl/boost_ladder_types.h>
#include <descartes_light/core/solver.h>
#include <descartes_light/core/edge_evaluator.h>
#include <descartes_light/solvers/bgl/boost_ladder_types.h>

//visitor will need access to edge evaluators, float type, ladder graph. Needs to be a member of the solver
namespace descartes_light
{
template <typename FloatType>
struct AddAllVisitor : boost::default_dijkstra_visitor
{
  AddAllVisitor(std::vector<EdgeEvaluator<FloatType>>& edge_eval,
          std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>& predecessor_map,
          std::vector<std::vector<VertexDesc<FloatType>>>& ladder_rungs)
          : eval(edge_eval),
            predecessors(predecessor_map),
            rungs(ladder_rungs)
       {
       }

  // must have copy constructor
  //AddAllVisitor() : AddAllVisitor

  void examine_vertex(VertexDesc<FloatType> u, bglgraph<FloatType> g)
  {
    int out_deg = boost::out_degree(u, g);
    // return if the vertex has any out edges
    if (out_deg == 0)
    {
      // check rung
      std::size_t next_rung = 0; // = predecessors.size(); //assuming that a dummy start node is still used -> This does not work bc predecessors preallocates based on vertices
      for (VertexDesc<FloatType> prev = predecessors.at(u); prev != u; u = prev, prev = predecessors.at(u))
      {
        ++next_rung;
      }
      FloatType cost;
      for (std::size_t s = 0; s < rungs[next_rung].size(); ++s)
      {
        std::pair<bool, FloatType> results =
            eval[static_cast<size_t>(s)].evaluate(*g[u].state, *g[rungs[next_rung][s]].state);
        if (results.first)
        {
          cost = results.second + g[rungs[next_rung][s]].cost;
          if (next_rung == 1) //this if can probably be captured in the dummy node edges
            cost += g[u].cost;
          VertexDesc<FloatType> sap = rungs[next_rung][s];
          boost::add_edge(u, sap, cost, g);
        }
      }
    }
    return;
  }

private:
  std::vector<EdgeEvaluator<FloatType>>& eval;
  std::map<VertexDesc<FloatType>, VertexDesc<FloatType>>& predecessors; //this needs to point to the same instance of the predecessor map the ssearch is updating
  std::vector<std::vector<VertexDesc<FloatType>>>& rungs;
};
} //descartes_light
#endif
