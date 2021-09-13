#ifndef DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_DESCARTES_SPACE_HPP
#define DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_DESCARTES_SPACE_HPP

#include "descartes_light/ompl/descartes_space.h"
#include "ompl/util/Exception.h"
#include <limits>
#include <cstdlib>

const double IDX_TO_IDX_DIST = 1;

template <typename FloatType>
void descartes_light::DescartesStateSampler<FloatType>::sampleUniform(ompl::base::State* state)
{
  // Get the ladder rung structure to have knowledge of the available sample space
  std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs =
      space_->as<descartes_light::DescartesStateSpace<FloatType>>()->getLadderRungs();
  // Sample a random, valid rung
  std::size_t rung = static_cast<std::size_t>(rng_.uniformInt(0, static_cast<int>(ladder_rungs.size() - 1)));
  // On this selected rung sample a random index
  std::size_t idx = static_cast<std::size_t>(rng_.uniformInt(0, static_cast<int>(ladder_rungs[rung].size() - 1)));
  // Store new vertex to the returned state
  state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->rung = rung;
  state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->idx = idx;
}

template <typename FloatType>
void descartes_light::DescartesStateSampler<FloatType>::sampleUniformNear(ompl::base::State* state,
                                                                          const ompl::base::State* near,
                                                                          const double /*distance*/)
{
  // Get the ladder rung structure to have knowledge of the available sample space
  std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs =
      space_->as<descartes_light::DescartesStateSpace<FloatType>>()->getLadderRungs();
  // Find the rung we are trying to sample next to
  long unsigned int rung =
      near->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->rung;
  // Randomly select if we should sample up or down a rung
  bool sample_up = rng_.uniformBool();
  long unsigned int new_rung;
  // Sample up if either it was randomly selected to sample up OR we are forced to if it is sampling near the first rung
  // AND ensure we aren't sampling on the last rung
  if ((sample_up || rung == 0) && rung != ladder_rungs.size() - 1)
    new_rung = rung + 1;
  else
    new_rung = rung - 1;
  // Find new index on this new sampled rung
  std::size_t new_idx =
      static_cast<std::size_t>(rng_.uniformInt(0, static_cast<int>(ladder_rungs[new_rung].size() - 1)));
  // Store new vertex to the returned state
  state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->rung = new_rung;
  state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->idx = new_idx;
  // Ensure the new sample is within the ladder graph (it shouldn't be possible to be outside)
  space_->enforceBounds(state);
}

template <typename FloatType>
void descartes_light::DescartesStateSampler<FloatType>::sampleGaussian(ompl::base::State* state,
                                                                       const ompl::base::State* mean,
                                                                       const double stdDev)
{
  // Get the ladder rung structure to have knowledge of the available sample space
  std::vector<std::vector<VertexDesc<FloatType>>> ladder_rungs =
      space_->as<descartes_light::DescartesStateSpace<FloatType>>()->getLadderRungs();
  // Find the distance between rungs to account for high stddev
  const double rung_2_rung_dist = space_->as<descartes_light::DescartesStateSpace<FloatType>>()->getRungToRungDist();
  // Find the rung we are trying to sample near
  double rung = static_cast<double>(
      mean->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->rung);
  // Determine new rung to be sampled on
  long unsigned int new_rung =
      static_cast<long unsigned int>(floor(rng_.gaussian(rung, stdDev / rung_2_rung_dist) + 0.5));
  // Make sure we are inside the state space
  if (new_rung >= ladder_rungs.size())
    new_rung = ladder_rungs.size() - 1;
  // Find new random index on this new sampled rung
  std::size_t new_idx =
      static_cast<std::size_t>(rng_.uniformInt(0, static_cast<int>(ladder_rungs[new_rung].size() - 1)));
  // Store new vertex to the returned state
  state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->rung = new_rung;
  state->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->idx = new_idx;
  // Ensure the new sample is within the ladder graph (it shouldn't be possible to be outside)
  space_->enforceBounds(state);
}

template <typename FloatType>
bool descartes_light::DescartesStateSpace<FloatType>::isDiscrete() const
{
  return true;  // This is a discrete state space
}

template <typename FloatType>
unsigned int descartes_light::DescartesStateSpace<FloatType>::getDimension() const
{
  return 2;  // This state space is 2-dimensional
}

template <typename FloatType>
double descartes_light::DescartesStateSpace<FloatType>::getMaximumExtent() const
{
  return rung_to_rung_dist_ * static_cast<double>(ladder_rungs_.size() + 1);
}

template <typename FloatType>
double descartes_light::DescartesStateSpace<FloatType>::getMeasure() const
{
  return static_cast<double>(ladder_rungs_.size());
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::enforceBounds(ompl::base::State* state) const
{
  long unsigned int rung = state->as<StateType>()->rung;
  // Ensure the rung is less than the number of rungs
  if (rung >= ladder_rungs_.size())
  {
    rung = ladder_rungs_.size() - 1;
    state->as<StateType>()->rung = rung;
  }
  // Ensure index in the rung exists by checking the rung length
  if (state->as<StateType>()->idx >= ladder_rungs_[rung].size())
  {
    state->as<StateType>()->idx = ladder_rungs_[rung].size() - 1;
  }
}

template <typename FloatType>
bool descartes_light::DescartesStateSpace<FloatType>::satisfiesBounds(const ompl::base::State* state) const
{
  long unsigned int rung = state->as<StateType>()->rung;
  if (rung >= ladder_rungs_.size())
    return false;
  if (state->as<StateType>()->idx >= ladder_rungs_[rung].size())
    return false;
  return true;
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::copyState(ompl::base::State* destination,
                                                                const ompl::base::State* source) const
{
  destination->as<StateType>()->rung = source->as<StateType>()->rung;
  destination->as<StateType>()->idx = source->as<StateType>()->idx;
}

template <typename FloatType>
unsigned int descartes_light::DescartesStateSpace<FloatType>::getSerializationLength() const
{
  return sizeof(long unsigned int) * 2;
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::serialize(void* serialization,
                                                                const ompl::base::State* state) const
{
  memcpy(serialization, &state->as<StateType>()->rung, sizeof(long unsigned int) * 2);
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::deserialize(ompl::base::State* state,
                                                                  const void* serialization) const
{
  memcpy(&state->as<StateType>()->rung, serialization, sizeof(long unsigned int) * 2);
}

template <typename FloatType>
double descartes_light::DescartesStateSpace<FloatType>::distance(const ompl::base::State* state1,
                                                                 const ompl::base::State* state2) const
{
  // Get the rungs and indexes to be measure
  long unsigned int rung1 = state1->as<StateType>()->rung;
  long unsigned int idx1 = state1->as<StateType>()->idx;
  long unsigned int rung2 = state2->as<StateType>()->rung;
  long unsigned int idx2 = state2->as<StateType>()->idx;

  // Initialize the distance between states at 0
  double dist = 0;

  // IF the states are equal then the distance is 0
  if (rung1 == rung2 && idx1 == idx2)
    return dist;

  // Determine the higher and lower value rungs to ensure we properly use the edge and vertex cost evaluators
  // The cost of the edge depends on the first rung and the cost of the vertex if from the vertex arrived at
  long unsigned int max_rung = std::max(rung1, rung2);
  long unsigned int min_rung = std::min(rung1, rung2);

  // Calculate the difference in rungs and indices
  long unsigned int rung_diff = max_rung - min_rung;
  int idx_diff = static_cast<int>(idx1) - static_cast<int>(idx2);

  // If they are on the same rung give a cost of 1 rung_to_rung_dist_, this should be seen as further away than points
  // on adjacent rungs, but closer than points separated by 1 rung in between
  if (rung_diff == 0)
    dist += rung_to_rung_dist_;

  // If the rung difference is greater than 1, find the rung difference, multiply by the distance per rung
  // Then add the distance between the indices to help with growing the trees randomly
  else if (rung_diff > 1)
    dist += rung_to_rung_dist_ * static_cast<double>(rung_diff) + IDX_TO_IDX_DIST * abs(idx_diff);

  // If the rungs are adjacent then proceed with evaluating the cost of going between them
  if (rung_diff == 1)
  {
    // If the min rung is the zeroth rung or the last added rung, then the cost should be 0. These states were added to
    // give a valid start and goal for the OMPL problem and should not be accounted for in the cost.
    if (min_rung == 0 || max_rung == ladder_rungs_.size() - 1)
      dist = 0;
    else
    {
      // Perform edge evaluation. Use the min_rung - 1 edge evaluator because there was an extra rung added to the graph
      // to provide a valid start state for OMPL which is not accounted for in the edge evaluator.
      std::pair<bool, FloatType> edge_res = edge_eval_[static_cast<std::size_t>(min_rung - 1)]->evaluate(
          *graph_[ladder_rungs_[rung1][idx1]].sample.state, *graph_[ladder_rungs_[rung2][idx2]].sample.state);

      // If this is a valid connection then the cost is added or else the cost of 1 rung distance is given
      if (edge_res.first)
        dist += static_cast<double>(edge_res.second);
      else
        dist += rung_to_rung_dist_;
    }
  }
  // Ensure that this isn't going to the last rung artificially added, then add the cost of the higher rung in the
  // connection
  if (max_rung != ladder_rungs_.size() - 1)
  {
    if (rung1 == max_rung)
      dist += graph_[ladder_rungs_[rung1][idx1]].sample.cost;
    else
      dist += graph_[ladder_rungs_[rung2][idx2]].sample.cost;
  }

  // If a cost of zero was found, add the distance epsilon because OMPL requires that the distance between any 2
  // distinct states is >0
  if (dist == 0)
    dist += distance_epsilon_;
  return dist;
}

template <typename FloatType>
bool descartes_light::DescartesStateSpace<FloatType>::equalStates(const ompl::base::State* state1,
                                                                  const ompl::base::State* state2) const
{
  long unsigned int rung1 = state1->as<StateType>()->rung;
  long unsigned int idx1 = state1->as<StateType>()->idx;
  long unsigned int rung2 = state2->as<StateType>()->rung;
  long unsigned int idx2 = state2->as<StateType>()->idx;
  if (rung1 == rung2 && idx1 == idx2)
    return true;
  return false;
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::interpolate(const ompl::base::State* from,
                                                                  const ompl::base::State* to,
                                                                  const double t,
                                                                  ompl::base::State* state) const
{
  // Return the from state if t=0
  if (t == 0)
  {
    state->as<StateType>()->rung = from->as<StateType>()->rung;
    state->as<StateType>()->idx = from->as<StateType>()->idx;
  }
  // Return the to state if t=1
  else if (t == 1)
  {
    state->as<StateType>()->rung = to->as<StateType>()->rung;
    state->as<StateType>()->idx = to->as<StateType>()->idx;
  }
  // Return the to state if from=to
  else if (equalStates(from, to))
  {
    state->as<StateType>()->rung = from->as<StateType>()->rung;
    state->as<StateType>()->idx = from->as<StateType>()->idx;
  }
  else
  {
    // Get the rungs and indexes to be measure
    long unsigned int rung1 = from->as<StateType>()->rung;
    long unsigned int idx1 = from->as<StateType>()->idx;
    long unsigned int rung2 = to->as<StateType>()->rung;
    long unsigned int idx2 = to->as<StateType>()->idx;
    int rung_diff = static_cast<int>(rung1) - static_cast<int>(rung2);
    long unsigned int new_rung = rung1;
    long unsigned int new_idx = idx1;

    // If initial rung difference is one, then return the from state because interpolation here can cause an endless
    // loop in RRTConnect
    if (abs(rung_diff) == 1)
    {
      state->as<StateType>()->rung = rung1;
      state->as<StateType>()->idx = idx1;
    }
    else
    {
      // If rungs are the same then try and sample up a rung if possible, otherwise sample down a rung
      if (rung_diff == 0)
      {
        if (rung1 != ladder_rungs_.size() - 1)
          new_rung = rung1 + 1;
        else if (rung1 != 0)
          new_rung = rung1 - 1;
        else
          throw ompl::Exception("Ladder graph a length of 1");
      }

      // Sample in the direction of rung2 relative to rung1
      else if (rung1 < rung2)
      {
        new_rung = rung1 + 1;
      }
      else
      {
        new_rung = rung1 - 1;
      }

      // Determine length of new rung
      long unsigned int new_rung_size = ladder_rungs_[new_rung].size();

      // Keep track to determine if we will return the from state or an actual new state
      bool found_point_under_max_dist = false;

      // Maximum rung between the first rung and the new rung
      long unsigned int max_rung = std::max(rung1, new_rung);
      long unsigned int min_rung = std::min(rung1, new_rung);

      // Iterate over new rung length
      for (std::size_t i = 0; i < new_rung_size; i++)
      {
        // Start and index of rung2 so that indices of lower values are not favored
        long unsigned int new_idx_test = i + idx2;
        // Circle back to 0 once we exceed rung size limits
        if (new_idx_test >= new_rung_size)
          new_idx_test -= new_rung_size;

        // If new rung is the first or last rung then we immediately know it has a cost of 0 and is valid
        if (min_rung == 0 || max_rung == ladder_rungs_.size() - 1)
        {
          new_idx = new_idx_test;
          found_point_under_max_dist = true;
          break;
        }

        // Cost evaluation
        std::pair<bool, FloatType> edge_res = edge_eval_[static_cast<std::size_t>(min_rung - 1)]->evaluate(
            *graph_[ladder_rungs_[rung1][idx1]].sample.state,
            *graph_[ladder_rungs_[new_rung][new_idx_test]].sample.state);
        double dist;
        // If this is a valid connection then the cost is added or else the cost of 1 rung distance is given
        if (edge_res.first)
        {
          dist = static_cast<double>(edge_res.second);
          if (max_rung != ladder_rungs_.size() - 1)
          {
            // Ensure that this isn't going to the last rung artificially added, then add the cost of the higher rung in
            // the connection
            if (rung1 == max_rung)
              dist += graph_[ladder_rungs_[rung1][idx1]].sample.cost;
            else
              dist += graph_[ladder_rungs_[new_rung][new_idx_test]].sample.cost;
          }
        }
        else
          dist = rung_to_rung_dist_;

        // If the cost is less than the max distance then we've found a valid index and should break out of the loop
        if (dist < max_dist_)
        {
          new_idx = new_idx_test;
          found_point_under_max_dist = true;
          break;
        }
      }

      // Create new vertex if a valid connection was found, otherwise return from state because no interpolation was
      // possible
      if (found_point_under_max_dist)
      {
        state->as<StateType>()->rung = new_rung;
        state->as<StateType>()->idx = new_idx;
      }
      else
      {
        state->as<StateType>()->rung = rung1;
        state->as<StateType>()->idx = idx1;
      }
    }
  }
}

template <typename FloatType>
ompl::base::StateSamplerPtr descartes_light::DescartesStateSpace<FloatType>::allocDefaultStateSampler() const
{
  return std::make_shared<descartes_light::DescartesStateSampler<FloatType>>(this);
}

template <typename FloatType>
ompl::base::State* descartes_light::DescartesStateSpace<FloatType>::allocState() const
{
  return new StateType();
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::freeState(ompl::base::State* state) const
{
  delete static_cast<StateType*>(state);
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
void descartes_light::DescartesStateSpace<FloatType>::printState(const ompl::base::State* state,
                                                                 std::ostream& out) const
{
  out << "DescartesState [";
  if (state != nullptr)
  {
    out << "rung " << state->as<StateType>()->rung;
    out << " idx " << state->as<StateType>()->idx;
  }
  else
    out << "nullptr";
  out << ']' << std::endl;
}

template <typename FloatType>
void descartes_light::DescartesStateSpace<FloatType>::printSettings(std::ostream& out) const
{
  out << "Descartes state space '" << getName() << "' with " << ladder_rungs_.size() << " rungs " << std::endl;
}

template <typename FloatType>
bool descartes_light::DescartesMotionValidator<FloatType>::checkMotion(const ompl::base::State* s1,
                                                                       const ompl::base::State* s2) const
{
  // Make sure the motion is increasing in rung value by 1 to be valid
  std::size_t rung1 = s1->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->rung;
  std::size_t rung2 = s2->as<typename descartes_light::DescartesStateSpace<FloatType>::StateType>()->rung;
  return rung2 > rung1 && rung2 - rung1 == 1;
}

template <typename FloatType>
bool descartes_light::DescartesMotionValidator<FloatType>::checkMotion(
    const ompl::base::State* /*s1*/,
    const ompl::base::State* /*s2*/,
    std::pair<ompl::base::State*, double>& /*lastValid*/) const
{
  return true;
}

#endif  // DESCARTES_LIGHT_SOLVERS_OMPL_IMPL_DESCARTES_SPACE_HPP
