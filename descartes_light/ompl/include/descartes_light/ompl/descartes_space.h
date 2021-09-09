#ifndef DESCARTES_LIGHT_SOLVERS_OMPL_DESCARTES_SPACE_H
#define DESCARTES_LIGHT_SOLVERS_OMPL_DESCARTES_SPACE_H

#include "ompl/base/StateSpace.h"
#include <ompl/base/MotionValidator.h>
#include <vector>
#include <string>
#include <map>

#include <descartes_light/bgl/bgl_solver.h>

using namespace ompl::base;

namespace descartes_light
{
    /** \brief State sampler for the Descartes state space */
    template <typename FloatType>
    class DescartesStateSampler : public ompl::base::StateSampler
    {
    public:
        /** \brief Constructor */
        DescartesStateSampler(const ompl::base::StateSpace *space) : StateSampler(space)
        {
        }

        /** \brief Sample a random state from the ladder grahp */
        void sampleUniform(ompl::base::State *state) override;

        /** \brief Sample a state such that each component state[i] is
            uniformly sampled from either adjacent rung of the current state.
            Note: there is no gaurentee that state sill be within the distance */
        void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

        /** \brief Sample a state such that each component state[i] has
            a Gaussian distribution with mean mean[i] and standard
            deviation stdDev. If the sampled value exceeds the state
            space boundary, it is thresholded to the nearest boundary. */
        void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;
    };

    /** \brief A state space representing the Descartes boost graph. The distance function is determined by the difference
     * in rungs and indices unless the rungs are adjacent, then an actual cost is made calculated using an edge evaluator */
    template <typename FloatType>
    class DescartesStateSpace : public ompl::base::StateSpace
    {
    public:

        /** \brief The definition of a state in the Descartes State Space */
        class StateType : public ompl::base::State
        {
        public:
            /** \brief The definition of the vertex (rung, idx) */
          std::pair<long unsigned int, long unsigned int> vertex;
        };

        /** \brief Constructor. The dimension of of the space needs to be specified. A space representing
            the Descartes State Space will be instantiated */
        DescartesStateSpace<FloatType>(descartes_light::BGLGraph<FloatType> graph,
                                       std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs,
                                       const std::vector<typename descartes_light::EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
                                       const double max_dist,
                                       const double rung_to_rung_dist = 100000)
          : graph_(graph)
          , ladder_rungs_(ladder_rungs)
          , edge_eval_(std::move(edge_eval))
          , max_dist_(max_dist)
          , rung_to_rung_dist_(rung_to_rung_dist)
          , stateBytes_(sizeof(std::pair<long unsigned int, long unsigned int>))
        {
            type_ = 14; // Larger than default types
            setName("Descartes" + getName());
        }

        ~DescartesStateSpace() override = default;

        /** \brief This state space is dicsrete which causes certain behaviors in OMPL */
        bool isDiscrete() const override;

        /** \brief Return the number of dimenstions in this space (2) */
        unsigned int getDimension() const override;

        /** \brief Get the maximum distance possible to record, this is the rung_to_rung_dist_*(number of rungs) */
        double getMaximumExtent() const override;

        /** \brief Returns the number of rungs in the ladder */
        double getMeasure() const override;

        /** \brief Ensures that the state is within the ladder graph */
        void enforceBounds(ompl::base::State *state) const override;

        /** \brief Checks if the state is within the ladder graph */
        bool satisfiesBounds(const ompl::base::State *state) const override;

        void copyState(ompl::base::State *destination, const ompl::base::State *source) const override;

        unsigned int getSerializationLength() const override;

        void serialize(void *serialization, const ompl::base::State *state) const override;

        void deserialize(ompl::base::State *state, const void *serialization) const override;

        /** \brief Measures the distance between two given states.
         * If they are on the same rung this will start with a base distance of 1 rung length apart.
         * If they are more than 1 rung apart it will have a base distance of rung_to_rung_dist_*(rung difference) + (index difference).
         * If they are 1 rung apart it will evaluate the cost of the edge.
         * The rung it is going towards will then have the cost of that vertex
         * added unless it is going to the goal vertex which has a cost of 0. */
        double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

        bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override;

        /** \brief This will interpolate to the best of its ability between the two states given.
         * If t=0 this will return the from input, if t=1 this will return the to input, if from=to then to will be returned.
         * Any other value of t will result in an interpolation to the first point found under the max_dist_ cost threshhold.
         * First it is found which side the to state is on relative to the from state, then a search is performed 1 rung to the
         * side relative to the from state. This adjacent rung is then sequentially searched starting at the idx provided by the
         * to state, a check if performed to ensure no sampling is attempted outside of the valid range for this searched rung.
         * Once a valid edge + state with a cost less than the max_dist_ parameter is found, this state is returned. */
        void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;

        ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;

        ompl::base::State *allocState() const override;

        void freeState(ompl::base::State *state) const override;

        void printState(const ompl::base::State *state, std::ostream &out) const override;

        void printSettings(std::ostream &out) const override;

        void setup() override;

        descartes_light::BGLGraph<FloatType> getGraph() const
        {
          return graph_;
        }

        std::vector<std::vector<VertexDesc<FloatType>>> getLadderRungs() const
        {
          return ladder_rungs_;
        }

        double getMaxDist() const
        {
          return max_dist_;
        }

        double getRungToRungDist() const
        {
          return rung_to_rung_dist_;
        }

        double getDistanceEpsilon() const
        {
          return distance_epsilon_;
        }

    protected:
        /** \brief Boost graph ladder, used to help with edge and vertex cost evaluation */
        descartes_light::BGLGraph<FloatType> graph_;
        /** @brief Ladder graph representation of the graph vertices, used for creating edge connections */
        std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs_;
        /** @brief Edge evaluator used to evaluate the cost of edges */
        const std::vector<typename descartes_light::EdgeEvaluator<FloatType>::ConstPtr> edge_eval_;
        /** @brief Maximum allowed cost for an edge to be valid */
        const double max_dist_;
        /** @brief The cost assigned to calculate a meaningful distance between non-adjacent rungs.
         * This value should be greater than any possible cost between adjacent rungs, however it should not
         * be MAX_VALUE because the number of rungs away needs to be meaningful
         * Example: The cost between rung 1 and rung 5 will be 4*rung_to_rung_dist_ */
        const double rung_to_rung_dist_;
        /** @brief This is the minimum assigned cost for when a edge is evaulated to have no cost
         * which is invalid for OMPL algorithms */
        const double distance_epsilon_ = 0.000001;

    private:
        /** \brief The size of a state, in bytes */
        std::size_t stateBytes_;
    };

    /** \brief A motion validator for the Descartes State Space */
    template <typename FloatType>
    class DescartesMotionValidator : public ompl::base::MotionValidator
    {
    public:
      DescartesMotionValidator(const ompl::base::SpaceInformationPtr& si)
        : MotionValidator(si.get())
      {
      }

      bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override;

      bool checkMotion(const ompl::base::State* s1,
                       const ompl::base::State* s2,
                       std::pair<ompl::base::State*, double>& lastValid) const override;

    protected:
      /** \brief The instance of space information this state validity checker operates on */
      SpaceInformation *si_;

    };
}

#endif // DESCARTES_LIGHT_SOLVERS_OMPL_DESCARTES_SPACE_H
