#include "descartes_light/edge_sampler.h"

descartes_light::DistanceEdgeEvaluator::DistanceEdgeEvaluator()
{

}

bool descartes_light::DistanceEdgeEvaluator::evaluate(const Rung_<double>& from, const Rung_<double>& to,
                                                      std::vector<LadderGraph<double>::EdgeList>& edges)
{
  return false;
}
