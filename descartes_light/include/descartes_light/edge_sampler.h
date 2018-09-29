#ifndef DESCARTES_LIGHT_EDGE_SAMPLER_H
#define DESCARTES_LIGHT_EDGE_SAMPLER_H

#include "descartes_light/ladder_graph.h"
#include <memory>

namespace descartes_light
{

class EdgeEvaluator
{
public:
  virtual ~EdgeEvaluator() {}

  virtual bool evaluate(const Rung_<double>& from, const Rung_<double>& to,
                        std::vector<LadderGraph<double>::EdgeList>& edges) = 0;
};

class DistanceEdgeEvaluator : public EdgeEvaluator
{
public:
  DistanceEdgeEvaluator(const std::vector<double>& velocity_limits);

  bool evaluate(const Rung_<double>& from, const Rung_<double>& to,
                          std::vector<LadderGraph<double>::EdgeList>& edges) override;

  std::vector<double> velocity_limits_;
};

using EdgeEvaluatorPtr = std::shared_ptr<EdgeEvaluator>;

}

#endif
