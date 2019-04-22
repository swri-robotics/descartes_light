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
typedef std::function<bool(const double* pose)> IsValidFn;

/**
 * @brief This is used for OPW Kinematics to get all solution given solution set.
 *
 * In the case of OPW kinematics it only returns solution between PI and -PI so if you want
 * solutions outside this range you must provide this function
 */
typedef std::function<std::vector<double>(const double* pose)> GetRedundentSolutionsFn;


inline static bool isWithinLimits(const double *vertex, const Eigen::MatrixX2d& limits)
{
  for (int i = 0; i < limits.rows(); ++i)
    if ((vertex[i] < limits(i, 0)) || (vertex[i] > limits(i, 1)))
      return false;

  return true;
}

/**
 * @brief OPW Kinematics only return solution between PI and -PI. Provided the limits it will append redundent solutions.
 * @param sol The current solution returned from OPW kinematics
 * @param limits The joint limits of the robot
 */
inline static std::vector<double> getOPWRedundentSolutions(const double *sol, const Eigen::MatrixX2d& limits)
{
  int dof = 6;
  double val;
  std::vector<double> redundent_sols;
  for (int i = 0; i < dof; ++i)
  {
    val = sol[i];
    while ((val-=(2*M_PI)) > limits(i, 0))
    {
      std::vector<double> new_sol(sol, sol + dof);
      new_sol[i] = val;
      redundent_sols.insert(redundent_sols.end(), new_sol.begin(), new_sol.end());
    }

    val = sol[i];
    while ((val+=(2*M_PI)) < limits(i, 1))
    {
      std::vector<double> new_sol(sol, sol + dof);
      new_sol[i] = val;
      redundent_sols.insert(redundent_sols.end(), new_sol.begin(), new_sol.end());
    }
  }

  return redundent_sols;
}

/**
 * @brief OPW Kinematics only return solution between PI and -PI. Provided the limits it will append redundent solutions.
 * @param sol The current solution returned from OPW kinematics (first two are the rail position)
 * @param limits The joint limits of the rail + robot
 */
inline static std::vector<double> getOPWRailedRedundentSolutions(const double *sol, const Eigen::MatrixX2d& limits)
{
  int dof = 8;
  double val;
  std::vector<double> redundent_sols;
  for (int i = 2; i < dof; ++i)
  {
    val = sol[i];
    while ((val-=(2*M_PI)) > limits(i, 0))
    {
      std::vector<double> new_sol(sol, sol + dof);
      new_sol[i] = val;
      redundent_sols.insert(redundent_sols.end(), new_sol.begin(), new_sol.end());
    }

    val = sol[i];
    while ((val+=(2*M_PI)) < limits(i, 1))
    {
      std::vector<double> new_sol(sol, sol + dof);
      new_sol[i] = val;
      redundent_sols.insert(redundent_sols.end(), new_sol.begin(), new_sol.end());
    }
  }

  return redundent_sols;
}

}
#endif // DESCARTES_LIGHT_SAMPLERS_UTILS_H
