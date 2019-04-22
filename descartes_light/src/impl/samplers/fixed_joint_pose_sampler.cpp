#include <descartes_light/impl/samplers/fixed_joint_pose_sampler.h>

namespace descartes_light
{

template<typename FloatType>
FixedJointPoseSampler<FloatType>::FixedJointPoseSampler(const std::vector<FloatType>& fixed_joint_position)
  : fixed_joint_position_(fixed_joint_position)
{

}

template<typename FloatType>
bool FixedJointPoseSampler<FloatType>::sample(std::vector<FloatType>& solution_set)
{
  solution_set.insert(solution_set.end(), fixed_joint_position_.begin(), fixed_joint_position_.end());
  return true;
}

// Explicit template specialization
template class FixedJointPoseSampler<float>;
template class FixedJointPoseSampler<double>;

} // namespace descartes_light
