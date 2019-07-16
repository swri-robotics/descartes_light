#ifndef DESCARTES_LIGHT_UTILS_H
#define DESCARTES_LIGHT_UTILS_H

#include <memory>
#include <functional>
#include <Eigen/Geometry>

namespace descartes_light
{

/**
 * @brief This is used for passing a function to descartes that filters out invalid solutions.
 *
 * Example: This would be used to filter out solution outside of joint limits.
 *
 */
template<typename FloatType>
using IsValidFn = std::function<bool(const FloatType*)>;

/**
 * @brief This is used for OPW Kinematics to get all solution given solution set.
 *
 * In the case of OPW kinematics it only returns solution between PI and -PI so if you want
 * solutions outside this range you must provide this function
 */
template<typename FloatType>
using GetRedundantSolutionsFn = std::function<std::vector<FloatType>(const FloatType*)>;

/**
 * @brief isWithinLimits
 * @param vertex
 * @param limits
 */
template<typename FloatType>
inline static bool isWithinLimits(const FloatType* vertex,
                                  const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>& limits)
{
  for (int i = 0; i < limits.rows(); ++i)
    if ((vertex[i] < limits(i, 0)) || (vertex[i] > limits(i, 1)))
      return false;

  return true;
}

} // namespace descartes_light

#endif // DESCARTES_LIGHT_SAMPLERS_UTILS_H
