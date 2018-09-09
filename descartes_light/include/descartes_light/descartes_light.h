#ifndef DESCARTES_LIGHT_H
#define DESCARTES_LIGHT_H

#include "descartes_light/ladder_graph.h"
#include "descartes_light/position_sampler.h"
#include "descartes_light/edge_sampler.h"

namespace descartes_light
{

class Solver
{
public:
  Solver(std::size_t dof);

  bool build(const std::vector<PositionSamplerPtr>& trajectory,
             EdgeEvaluatorPtr edge_eval);

  bool search(std::vector<double>& solution);

private:
  LadderGraph<double> graph_;
};

}

#endif
