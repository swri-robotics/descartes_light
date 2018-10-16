#ifndef HYBRID_PLANNING_LOAD_ENV_H
#define HYBRID_PLANNING_LOAD_ENV_H

#include <tesseract_ros/kdl/kdl_env.h>

namespace hybrid_planning_common
{

/**
 * @brief Helper function to pull the environment from the ROS parameter's "robot_description" and
 * "robot_description_semantic" for the URDF and SRDF respectively.
 */
bool loadEnvironment(tesseract::tesseract_ros::KDLEnvPtr& env);

}

#endif
