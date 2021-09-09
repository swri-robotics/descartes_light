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
    /** \brief State sampler for the R<sup>n</sup> state space */
    template <typename FloatType>
    class DescartesStateSampler : public ompl::base::StateSampler
    {
    public:
        /** \brief Constructor */
        DescartesStateSampler(const ompl::base::StateSpace *space) : StateSampler(space)
        {
        }

        void sampleUniform(ompl::base::State *state) override;
        /** \brief Sample a state such that each component state[i] is
            uniformly sampled from [near[i]-distance, near[i]+distance].
            If this interval exceeds the state space bounds, the
            interval is truncated. */
        void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;
        /** \brief Sample a state such that each component state[i] has
            a Gaussian distribution with mean mean[i] and standard
            deviation stdDev. If the sampled value exceeds the state
            space boundary, it is thresholded to the nearest boundary. */
        void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;
    };

    /** \brief A state space representing R<sup>n</sup>. The distance function is the L2 norm. */
    template <typename FloatType>
    class DescartesStateSpace : public ompl::base::StateSpace
    {
    public:

        /** \brief The definition of a state in R<sup>n</sup> */
        class StateType : public ompl::base::State
        {
        public:
            /** \brief The value of the vertex */
          std::pair<long unsigned int, long unsigned int> vertex;
        };

        /** \brief Constructor. The dimension of of the space needs to be specified. A space representing
            R<sup>dim</sup> will be instantiated */
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

        bool isDiscrete() const override;

        unsigned int getDimension() const override;

        double getMaximumExtent() const override;

        double getMeasure() const override;

        void enforceBounds(ompl::base::State *state) const override;

        bool satisfiesBounds(const ompl::base::State *state) const override;

        void copyState(ompl::base::State *destination, const ompl::base::State *source) const override;

        unsigned int getSerializationLength() const override;

        void serialize(void *serialization, const ompl::base::State *state) const override;

        void deserialize(ompl::base::State *state, const void *serialization) const override;

        double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

        bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override;

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

        const double getMaxDist() const
        {
          return max_dist_;
        }

        const double getRungToRungDist() const
        {
          return rung_to_rung_dist_;
        }

        const double getDistanceEpsilon() const
        {
          return distance_epsilon_;
        }

    protected:
        /** \brief Boost graph ladder */
        descartes_light::BGLGraph<FloatType> graph_;
        /** @brief Ladder graph representation of the graph vertices, used for creating edge connections */
        std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs_;

        const std::vector<typename descartes_light::EdgeEvaluator<FloatType>::ConstPtr> edge_eval_;

        const double max_dist_;

        const double rung_to_rung_dist_;

        const double distance_epsilon_ = 0.000001;

    private:
        /** \brief The size of a state, in bytes */
        std::size_t stateBytes_;
    };

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
