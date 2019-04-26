#ifndef DESCARTES_OPW_UTILS_H
#define DESCARTES_OPW_UTILS_H

#include <Eigen/Geometry>

namespace descartes_light
{

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

} // namespace descartes_light

#endif // DESCARTES_OPW_UTILS_H
