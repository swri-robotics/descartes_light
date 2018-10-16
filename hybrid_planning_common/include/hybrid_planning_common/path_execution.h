#ifndef HYBRID_PLANNING_LOAD_ENV_H
#define HYBRID_PLANNING_LOAD_ENV_H

#include <trajectory_msgs/JointTrajectory.h>

namespace hybrid_planning_common
{

/**
 * @brief Helper to execute a given trajectory on the 'joint_trajectory_action' of type
 * 'control_msgs::FollowJointTrajectoryAction'. Blocks until the trajectory goal terminates.
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

}

#endif
