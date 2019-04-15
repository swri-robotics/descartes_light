#ifndef DESCARTES_LIGHT_SAMPLERS_UTILS_H
#define DESCARTES_LIGHT_SAMPLERS_UTILS_H

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

template<typename FloatType>
inline static bool isWithinLimits(const FloatType* vertex,
                                  const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>& limits)
{
  for (int i = 0; i < limits.rows(); ++i)
    if ((vertex[i] < limits(i, 0)) || (vertex[i] > limits(i, 1)))
      return false;

  return true;
}

/**
 * @brief OPW Kinematics only return solution between PI and -PI. Provided the limits it will append redundant solutions.
 * @param sol The current solution returned from OPW kinematics
 * @param limits The joint limits of the robot
 */
template<typename FloatType>
inline static std::vector<FloatType> getOPWRedundantSolutions(const FloatType* sol,
                                                              const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>& limits)
{
  int dof = 6;
  FloatType val;
  std::vector<FloatType> redundant_sols;
  for (int i = 0; i < dof; ++i)
  {
    val = sol[i];
    while ((val-=(2*M_PI)) > limits(i, 0))
    {
      std::vector<FloatType> new_sol(sol, sol + dof);
      new_sol[i] = val;
      redundant_sols.insert(redundant_sols.end(), new_sol.begin(), new_sol.end());
    }

    val = sol[i];
    while ((val+=(static_cast<FloatType>(2.0 * M_PI))) < limits(i, 1))
    {
      std::vector<FloatType> new_sol(sol, sol + dof);
      new_sol[i] = val;
      redundant_sols.insert(redundant_sols.end(), new_sol.begin(), new_sol.end());
    }
  }

  return redundant_sols;
}

/**
 * @brief OPW Kinematics only return solution between PI and -PI. Provided the limits it will append redundant solutions.
 * @param sol The current solution returned from OPW kinematics (first two are the rail position)
 * @param limits The joint limits of the rail + robot
 */
template<typename FloatType>
inline static std::vector<FloatType> getOPWRailedRedundantSolutions(const FloatType *sol,
                                                                    const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>& limits)
{
  int dof = 8;
  FloatType val;
  std::vector<FloatType> redundant_sols;
  for (int i = 2; i < dof; ++i)
  {
    val = sol[i];
    while ((val -= (static_cast<FloatType>(2.0 * M_PI))) > limits(i, 0))
    {
      std::vector<FloatType> new_sol(sol, sol + dof);
      new_sol[i] = val;
      redundant_sols.insert(redundant_sols.end(), new_sol.begin(), new_sol.end());
    }

    val = sol[i];
    while ((val+=(2*M_PI)) < limits(i, 1))
    {
      std::vector<FloatType> new_sol(sol, sol + dof);
      new_sol[i] = val;
      redundant_sols.insert(redundant_sols.end(), new_sol.begin(), new_sol.end());
    }
  }

  return redundant_sols;
}

}
#endif // DESCARTES_LIGHT_SAMPLERS_UTILS_H
