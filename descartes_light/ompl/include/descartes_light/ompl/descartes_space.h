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
//            long unsigned int value; // vertex descriptor of floattype
//            descartes_light::Vertex<FloatType> vertex;
          std::pair<long unsigned int, long unsigned int> vertex;
        };

        /** \brief Constructor. The dimension of of the space needs to be specified. A space representing
            R<sup>dim</sup> will be instantiated */
        DescartesStateSpace<FloatType>(descartes_light::BGLGraph<FloatType> graph,
                                       std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs,
                                       const std::vector<typename descartes_light::EdgeEvaluator<FloatType>::ConstPtr>& edge_eval,
                                       double max_dist)
          : graph_(graph), ladder_rungs_(ladder_rungs), edge_eval_(std::move(edge_eval)), max_dist_(max_dist), stateBytes_(sizeof(long unsigned int))
        {
            type_ = 14; // Larger than default types
            setName("Descartes" + getName());
        }

        ~DescartesStateSpace() override = default;

        bool isDiscrete() const override;

//            /** \brief Increase the dimensionality of the state space by 1. Optionally, bounds can be specified for this
//             * added dimension. setup() will need to be called after adding dimensions. */
//            void addDimension(double minBound = 0.0, double maxBound = 0.0);

//            /** \brief Increase the dimensionality of the state space by 1 and specify the name of this dimension.
//             * Optionally, bounds can be specified for this added dimension. setup() will need to be called after adding
//             * dimensions. This function is a wrapper for the previous definition of addDimension(). */
//            void addDimension(const std::string &name, double minBound = 0.0, double maxBound = 0.0);

//            /** \brief Set the bounds of this state space. This defines
//                the range of the space in which sampling is performed. */
//            void setBounds(const RealVectorBounds &bounds);

//            /** \brief Set the bounds of this state space. The bounds for
//                each dimension will be the same: [\e low, \e high]. */
//            void setBounds(double low, double high);

//            /** \brief Get the bounds for this state space */
//            const RealVectorBounds &getBounds() const
//            {
//                return bounds_;
//            }

        unsigned int getDimension() const override;

//            /** \brief Each dimension can optionally have a name associated to it. If it does, this function returns
//               that name.
//                Return empty string otherwise */
//            const std::string &getDimensionName(unsigned int index) const;

//            /** \brief Get the index of a specific dimension, by name. Return -1 if name is not found */
//            int getDimensionIndex(const std::string &name) const;

//            /** \brief Set the name of a dimension */
//            void setDimensionName(unsigned int index, const std::string &name);

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

//        double *getValueAddressAtIndex(ompl::base::State *state, unsigned int index) const override;

        void printState(const ompl::base::State *state, std::ostream &out) const override;

        void printSettings(std::ostream &out) const override;

        void registerProjections() override;

        void setup() override;

        std::vector<std::vector<VertexDesc<FloatType>>> getLadderRungs() const
        {
          return ladder_rungs_;
        }

    protected:
        /** \brief Boost graph ladder */
        descartes_light::BGLGraph<FloatType> graph_;
        /** @brief Ladder graph representation of the graph vertices, used for creating edge connections */
        std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs_;

        const std::vector<typename descartes_light::EdgeEvaluator<FloatType>::ConstPtr> edge_eval_;

        double max_dist_;

    private:
        /** \brief The size of a state, in bytes */
        std::size_t stateBytes_;
    };

    template <typename FloatType>
    class DescartesMotionValidator : public ompl::base::MotionValidator
    {
    public:
//      DescartesMotionValidator(ompl::base::SpaceInformationPtr *si) : si_(si), valid_(0), invalid_(0)
//      {
//      }
//      DescartesMotionValidator(const ompl::base::SpaceInformationPtr& si) : si_(si.get()), valid_(0), invalid_(0)
//      {
//      }
      DescartesMotionValidator(const ompl::base::SpaceInformationPtr& si)
        : MotionValidator(si.get())
      {
      }
//      DescartesMotionValidator()
//      {
//      }

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
