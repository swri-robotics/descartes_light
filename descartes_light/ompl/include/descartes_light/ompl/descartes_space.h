#ifndef DESCARTES_LIGHT_SOLVERS_OMPL_DESCARTES_SPACE_H
#define DESCARTES_LIGHT_SOLVERS_OMPL_DESCARTES_SPACE_H

#include <ompl/base/StateSpace.h>
#include <ompl/base/MotionValidator.h>
#include <vector>
#include <string>
#include <map>

#include <descartes_light/solvers/ladder_graph/ladder_graph.h>
#include <descartes_light/core/solver.h>

namespace descartes_light
{
/** \brief State sampler for the Descartes state space */
template <typename FloatType>
class DescartesStateSampler : public ompl::base::StateSampler
{
public:
  /** \brief Constructor */
  DescartesStateSampler(const ompl::base::StateSpace* space) : StateSampler(space) {}

  /** \brief Sample a random state from the ladder grahp */
  void sampleUniform(ompl::base::State* state) override;

  /** \brief Sample a state such that each component state[i] is
      uniformly sampled from either adjacent rung of the current state.
      Note: there is no gaurentee that state sill be within the distance */
  void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, double distance) override;

  /** \brief Sample a state such that each component state[i] has
      a Gaussian distribution with mean mean[i] and standard
      deviation stdDev. If the sampled value exceeds the state
      space boundary, it is thresholded to the nearest boundary. */
  void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, double stdDev) override;
};

/** \brief An OMPL state space representing the Descartes planning graph.
 * \details The graph is composed of several "rungs" of vertices. A successful path through the graph visits exactly one
 * vertex on each rung in the correct sequential order. Each rung represents a waypoint and the vertices in a rung
 * represent the possible ways that that waypoint can be properly reached. A state in this space is reprsented by two
 * integers, reprsenting the rung and index of a given vertex. The distance function between any two given states on
 * non-adjacent rungs is determined by the difference in rungs multiplied by a large value, rung_to_rung_dist_, summed
 * with the difference in indices. If the rungs are adjacent an actual cost is calculated using an edge evaluator. The
 * interpolation algorithm gurantees that the interpolated point will be on an adjacent rung next to the desired
 * starting state if there exists a valid transition with a cost below the given threshold, max_dist_ */
template <typename FloatType>
class DescartesStateSpace : public ompl::base::StateSpace
{
public:
  /** \brief The definition of a state in the Descartes State Space */
  class StateType : public ompl::base::State
  {
  public:
    /** \brief The rung of the point of interest */
    long unsigned int rung;
    /** \brief The index of the point of interest */
    long unsigned int idx;
  };

  /** \brief Constructor. The dimension of of the space needs to be specified. A space representing
      the Descartes State Space will be instantiated */
  DescartesStateSpace<FloatType>(
      LadderGraph<FloatType> graph,
      const std::vector<typename descartes_light::EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
      const double max_dist)
    : graph_(graph)
    , edge_eval_(std::move(edge_eval))
    , max_dist_(max_dist)
    , rung_to_rung_dist_(max_dist * 10000)
    , distance_epsilon_(std::numeric_limits<double>::epsilon())
  {
    type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 1;  // Larger than default types
    setName("Descartes" + getName());
  }

  ~DescartesStateSpace() override = default;

  /** \brief This state space is dicsrete which causes certain behaviors in OMPL */
  bool isDiscrete() const override;

  /** \brief Return the number of dimenstions in this space (2) */
  unsigned int getDimension() const override;

  /** \brief Get the maximum distance possible to record, this is the rung_to_rung_dist_*(number of rungs) */
  double getMaximumExtent() const override;

  /** \brief Returns the number of vertices in the graph (a representation of volume of the space) */
  double getMeasure() const override;

  /** \brief Ensures that the state is within the ladder graph */
  void enforceBounds(ompl::base::State* state) const override;

  /** \brief Checks if the state is within the ladder graph */
  bool satisfiesBounds(const ompl::base::State* state) const override;

  void copyState(ompl::base::State* destination, const ompl::base::State* source) const override;

  unsigned int getSerializationLength() const override;

  void serialize(void* serialization, const ompl::base::State* state) const override;

  void deserialize(ompl::base::State* state, const void* serialization) const override;

  /** \brief Measures the distance between two given states.
   * Equal states return a distance of 0.0
   * If they are on the same rung this will start with a base distance of 1 rung length apart.
   * If they are more than 1 rung apart it will have a base distance of rung_to_rung_dist_*(rung difference) + (index
   * difference).
   * If they are 1 rung apart it will evaluate the cost of the edge between the two states. The rung it is going towards
   * will then have the cost of that vertex added, unless it is going to the goal vertex which has a cost of 0. Ideally
   * this cost will evaluate to less than the rung_to_rung_dist_ value */
  double distance(const ompl::base::State* state1, const ompl::base::State* state2) const override;

  bool equalStates(const ompl::base::State* state1, const ompl::base::State* state2) const override;

  /** \brief Interpolates between two states.
   * If t=0, returns the "from" state
   * If t=1, returns the "to" state
   * If "from"=to" returns "from"
   * 0 < t < 1 results in an interpolation to the first state found under the max_dist_ cost threshhold, this state is
   * found by the following process:
   * First it is found which side the "to" state is on relative to the "from" state then a search is performed 1 rung to
   * the side relative to the "from" state. For example, if the initial states were, [rung,index], from=[1,2] and
   * to=[50,3] the inerpolation would be done by searching rung 2 looking for an edge from [1,2] to [2,x] that evaluates
   * to a cost < max_dist_. This adjacent rung is sequentially searched starting at the idx provided by the "to" state,
   * a check is performed to ensure no sampling is attempted outside of the valid range for this searched rung. Once a
   * valid edge with a cost less than the max_dist_ parameter is found, this state is returned.
   * Note, this does not mean that the optimal transition will be found from the "from" state, just a sufficient one */
  void interpolate(const ompl::base::State* from,
                   const ompl::base::State* to,
                   double t,
                   ompl::base::State* state) const override;

  ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;

  ompl::base::State* allocState() const override;

  void freeState(ompl::base::State* state) const override;

  void printState(const ompl::base::State* state, std::ostream& out) const override;

  void printSettings(std::ostream& out) const override;

  void setup() override;

  const LadderGraph<FloatType>& getGraph() const { return graph_; }

  double getMaxDist() const { return max_dist_; }

  double getRungToRungDist() const { return rung_to_rung_dist_; }

  double getDistanceEpsilon() const { return distance_epsilon_; }

protected:
  LadderGraph<FloatType> graph_;
  /** @brief Edge evaluator used to evaluate the cost of edges */
  const std::vector<typename descartes_light::EdgeEvaluator<FloatType>::ConstPtr> edge_eval_;
  /** @brief Maximum allowed cost for an edge to be valid.
   * This will be unique to each for each problem and could potentially vary significantly between problem setups.
   * Large values will lead to fast, but highly non-optimal solutions.
   * Very small values will be impossible to solve and never converge.
   * Values correctly calibrated for a given problem can lead to near optimal solutions, but can take longer. */
  const double max_dist_;
  /** @brief A large cost assigned to calculate a meaningful distance between non-adjacent rungs.
   * This value should ideally be much greater than max_dist_, however it should not
   * be MAX_VALUE because the number of rungs away needs to be meaningful
   * Example: The cost between rung 1 and rung 5 will be 4*rung_to_rung_dist_ */
  const double rung_to_rung_dist_;
  /** @brief This is the minimum assigned cost for when a edge is evaulated to have no cost
   * which is invalid for OMPL algorithms */
  const double distance_epsilon_;
  ;
};

/** \brief Ensures only motions that increment by +1 rungs are valid */
template <typename FloatType>
class DescartesMotionValidator : public ompl::base::MotionValidator
{
public:
  DescartesMotionValidator(const ompl::base::SpaceInformationPtr& si) : MotionValidator(si.get()) {}

  bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override;

  bool checkMotion(const ompl::base::State* s1,
                   const ompl::base::State* s2,
                   std::pair<ompl::base::State*, double>& lastValid) const override;

protected:
  /** \brief The instance of space information this state validity checker operates on */
  ompl::base::SpaceInformation* si_;
};
}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_SOLVERS_OMPL_DESCARTES_SPACE_H
