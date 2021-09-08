#include "descartes_light/ompl/descartes_space.h"
#include "ompl/util/Exception.h"
#include <limits>
#include <cstdlib>

const double RUNG_TO_RUNG_DIST = 10000;
const double IDX_TO_IDX_DIST = 1;

template <typename FloatType>
void descartes_light::DescartesStateSampler<FloatType>::sampleUniform(ompl::base::State *state)
{
    std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs =
        space_->as<descartes_light::DescartesStateSpace<FloatType>>()->getLadderRungs();
    std::size_t rung = static_cast<std::size_t>(rng_.uniformInt(0, static_cast<int>(ladder_rungs.size() - 1)));
    std::size_t idx = static_cast<std::size_t>(rng_.uniformInt(0, static_cast<int>(ladder_rungs[rung].size() - 1)));
    std::pair<long unsigned int, long unsigned int> vertex(rung, idx);
    state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex = vertex;
}

template <typename FloatType>
void descartes_light::DescartesStateSampler<FloatType>::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
{
  std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs =
      space_->as<descartes_light::DescartesStateSpace<FloatType>>()->getLadderRungs();
  long unsigned int rung = near->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.first;
  bool sample_up = rng_.uniformBool();
  long unsigned int new_rung;
  if ((sample_up || rung == 0) && rung != ladder_rungs.size() - 1)
    new_rung = rung + 1;
  else
    new_rung = rung - 1;
  std::size_t new_idx = static_cast<std::size_t>(rng_.uniformInt(0, static_cast<int>(ladder_rungs[new_rung].size() - 1)));
  std::pair<long unsigned int, long unsigned int> new_vertex(new_rung, new_idx);
  state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex = new_vertex;
  space_->enforceBounds(state);
}

template <typename FloatType>
void descartes_light::DescartesStateSampler<FloatType>::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
{
  std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs =
      space_->as<descartes_light::DescartesStateSpace<FloatType>>()->getLadderRungs();
  double rung = static_cast<double>(mean->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.first);
  long unsigned int new_rung = static_cast<long unsigned int>(floor(rng_.gaussian(rung, stdDev / RUNG_TO_RUNG_DIST) + 0.5));
  if (new_rung >= ladder_rungs.size())
    new_rung = ladder_rungs.size() - 1;
  std::size_t new_idx = static_cast<std::size_t>(rng_.uniformInt(0, static_cast<int>(ladder_rungs[new_rung].size() - 1)));
  std::pair<long unsigned int, long unsigned int> new_vertex(new_rung, new_idx);
  state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex = new_vertex;
  space_->enforceBounds(state);
}

template <typename FloatType>
bool descartes_light::DescartesStateSpace<FloatType>::isDiscrete() const
{
    return true;
}

template <typename FloatType>
unsigned int descartes_light::DescartesStateSpace<FloatType>::getDimension() const
{
    return 2;
}

template <typename FloatType>
double descartes_light::DescartesStateSpace<FloatType>::getMaximumExtent() const
{
    return RUNG_TO_RUNG_DIST * static_cast<double>(ladder_rungs_.size() + 1);
}

template <typename FloatType>
double descartes_light::DescartesStateSpace<FloatType>::getMeasure() const
{
    return static_cast<double>(ladder_rungs_.size());
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::enforceBounds(ompl::base::State *state) const
{
  long unsigned int rung = state->as<StateType>()->vertex.first;
  if (rung >= ladder_rungs_.size())
  {
    rung = ladder_rungs_.size() - 1;
    state->as<StateType>()->vertex.first = rung;
  }
  if (state->as<StateType>()->vertex.second >= ladder_rungs_[rung].size())
  {
    state->as<StateType>()->vertex.second = ladder_rungs_[rung].size() - 1;
  }
//    if (state->as<StateType>()->value < lowerBound_)
//        state->as<StateType>()->value = lowerBound_;
//    else if (state->as<StateType>()->value > upperBound_)
//        state->as<StateType>()->value = upperBound_;
}

template <typename FloatType>
bool descartes_light::DescartesStateSpace<FloatType>::satisfiesBounds(const ompl::base::State *state) const
{
  long unsigned int rung = state->as<StateType>()->vertex.first;
  if (rung >= ladder_rungs_.size())
    return false;
  if (state->as<StateType>()->vertex.second >= ladder_rungs_[rung].size())
    return false;
  return true;
//    return state->as<StateType>()->value >= lowerBound_ && state->as<StateType>()->value <= upperBound_;
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::copyState(ompl::base::State *destination, const ompl::base::State *source) const
{
    destination->as<StateType>()->vertex = source->as<StateType>()->vertex;
}

template <typename FloatType>
unsigned int descartes_light::DescartesStateSpace<FloatType>::getSerializationLength() const
{
    return sizeof(std::pair<long unsigned int, long unsigned int>);
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::serialize(void *serialization, const ompl::base::State *state) const
{
    memcpy(serialization, &state->as<StateType>()->vertex, sizeof(std::pair<long unsigned int, long unsigned int>));
//    memcpy(serialization, &state->as<StateType>()->value, sizeof(int));
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::deserialize(ompl::base::State *state, const void *serialization) const
{
    memcpy(&state->as<StateType>()->vertex, serialization, sizeof(std::pair<long unsigned int, long unsigned int>));
//    memcpy(&state->as<StateType>()->value, serialization, sizeof(int));
}

template <typename FloatType>
double descartes_light::DescartesStateSpace<FloatType>::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    long unsigned int rung1 = state1->as<StateType>()->vertex.first;
    long unsigned int idx1 = state1->as<StateType>()->vertex.second;
    long unsigned int rung2 = state2->as<StateType>()->vertex.first;
    long unsigned int idx2 = state2->as<StateType>()->vertex.second;
    int rung_diff = static_cast<int>(rung1) - static_cast<int>(rung2);
    int idx_diff = static_cast<int>(idx1) - static_cast<int>(idx2);
    double dist = 0;
    if (rung1 == rung2 && idx1 == idx2)
      return dist;
    if (abs(rung_diff) == 0)
      dist += RUNG_TO_RUNG_DIST;
    else if (abs(rung_diff) > 1)
      dist += RUNG_TO_RUNG_DIST * abs(rung_diff);
    if (abs(rung_diff) == 1)
    {
      // ADD edge eval
      dist += IDX_TO_IDX_DIST * abs(idx_diff);
    }
    else
      dist += IDX_TO_IDX_DIST * abs(idx_diff);
    if (dist == 0)
      dist += 0.1;
    return dist;
}

template <typename FloatType>
bool descartes_light::DescartesStateSpace<FloatType>::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const
{
  long unsigned int rung1 = state1->as<StateType>()->vertex.first;
  long unsigned int idx1 = state1->as<StateType>()->vertex.second;
  long unsigned int rung2 = state2->as<StateType>()->vertex.first;
  long unsigned int idx2 = state2->as<StateType>()->vertex.second;
  if (rung1 == rung2 && idx1 == idx2)
    return true;
  return false;
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
{
  if (t == 0)
    state->as<StateType>()->vertex = from->as<StateType>()->vertex;
  else if (t == 1)
    state->as<StateType>()->vertex = to->as<StateType>()->vertex;
  else if (equalStates(from, to))
    state->as<StateType>()->vertex = to->as<StateType>()->vertex;
  else
  {
    long unsigned int rung1 = from->as<StateType>()->vertex.first;
    long unsigned int idx1 = from->as<StateType>()->vertex.second;
    long unsigned int rung2 = to->as<StateType>()->vertex.first;
    long unsigned int idx2 = to->as<StateType>()->vertex.second;
    int rung_diff = static_cast<int>(rung1) - static_cast<int>(rung2);
    long unsigned int new_rung = rung1;
    long unsigned int new_idx = idx1;
    if (rung_diff = 0)
    {
      if (rung1 != ladder_rungs_.size() - 1)
        new_rung = rung1 + 1;
      else if (rung1 != 0)
        new_rung = rung1 - 1;
      else
        throw ompl::Exception("Ladder graph a length of 1");
    }
    else if (rung1 < rung2)
    {
      new_rung = rung1 + 1;
    }
    else
    {
      new_rung = rung1 - 1;
    }
    double maxD = t * distance(from, to);
    long unsigned int new_rung_size = ladder_rungs_[new_rung].size();
    for (std::size_t i = 0; i < new_rung_size; i++)
    {
      long unsigned int new_idx_test = i + idx1;
      if (new_idx_test >= new_rung_size)
        new_idx_test -= new_rung_size;
      // DITANCE CALCULATION
      int idx_diff = static_cast<int>(idx1) - static_cast<int>(new_idx_test);
      if (idx_diff < maxD)
      {
        new_idx = i;
        break;
      }
    }
    std::pair<long unsigned int, long unsigned int> new_vertex(new_rung, new_idx);
    state->as<StateType>()->vertex = new_vertex;
  }
//    state->as<StateType>()->value = (int)floor(from->as<StateType>()->value +
//                                               (to->as<StateType>()->value - from->as<StateType>()->value) * t + 0.5);
}

template <typename FloatType>
ompl::base::StateSamplerPtr descartes_light::DescartesStateSpace<FloatType>::allocDefaultStateSampler() const
{
    return std::make_shared<descartes_light::DescartesStateSampler<FloatType>>(this);
}

template <typename FloatType>
ompl::base::State *descartes_light::DescartesStateSpace<FloatType>::allocState() const
{
    return new StateType();
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::freeState(ompl::base::State *state) const
{
    delete static_cast<StateType *>(state);
}


//double *descartes_light::DescartesStateSpace::getValueAddressAtIndex(State *state, const unsigned int index) const
//{
//    return index < dimension_ ? static_cast<StateType *>(state)->values + index : nullptr;
//}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::registerProjections()
{
//    class DiscreteDefaultProjection : public ProjectionEvaluator
//    {
//    public:
//        DiscreteDefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
//        {
//        }

//        unsigned int getDimension() const override
//        {
//            return 1;
//        }

//        void defaultCellSizes() override
//        {
//            bounds_.resize(1);
//            bounds_.low[0] = space_->as<DescartesStateSpace>()->lowerBound_;
//            bounds_.high[0] = space_->as<DescartesStateSpace>()->upperBound_;
//            cellSizes_.resize(1);
//            cellSizes_[0] = 1.0;
//        }

//        void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
//        {
//            projection(0) = state->as<DescartesStateSpace::StateType>()->value;
//        }
//    };

//    registerDefaultProjection(std::make_shared<DiscreteDefaultProjection>(this));
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::setup()
{
    if (ladder_rungs_.size() == 0)
        throw ompl::Exception("Ladder rungs cannot be empty");
    if (ladder_rungs_.size() == 1)
        throw ompl::Exception("Ladder rungs cannot be length 1");
    ompl::base::StateSpace::setup();
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::printState(const ompl::base::State *state, std::ostream &out) const
{
    out << "DescartesState [";
    if (state != nullptr)
    {
        out << "rung " << state->as<StateType>()->vertex.first;
        out << " idx " << state->as<StateType>()->vertex.second;
    }
    else
        out << "nullptr";
    out << ']' << std::endl;
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::printSettings(std::ostream &out) const
{
    out << "Descartes state space '" << getName() << "' with " << ladder_rungs_.size() << " rungs "
        << std::endl;
}

template <typename FloatType>
bool descartes_light::DescartesMotionValidator<FloatType>::checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  std::size_t rung1 = s1->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.first;
  std::size_t rung2 = s2->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.first;
  return rung2 > rung1 && rung2 - rung1 == 1;
//  return true;
}

template <typename FloatType>
bool descartes_light::DescartesMotionValidator<FloatType>::checkMotion(const ompl::base::State* s1,
                                                                       const ompl::base::State* s2,
                                                                       std::pair<ompl::base::State*, double>& lastValid) const
{
//  std::size_t rung1 = s1->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.first;
//  std::size_t rung2 = s2->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex.first;
//  if (rung2 <= rung1 || rung2 - rung1 != 1)
//  {
//    lastValid.first->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex =
//        s1->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex;
//    lastValid.second = 0.0;
//    return false;
//  }
//  lastValid.first->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex =
//      s2->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->vertex;
//  lastValid.second = 1.0;
  return true;
}

