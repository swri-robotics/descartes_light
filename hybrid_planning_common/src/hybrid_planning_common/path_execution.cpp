#include "hybrid_planning_common/path_execution.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

bool hybrid_planning_common::executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac ("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);

  return ac.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool hybrid_planning_common::executeTrajectory(const hybrid_planning_common::JointPath& trajectory)
{
  for (const auto& path : trajectory)
  {
    if (!executeTrajectory(path)) return false;
  }
  return true;
}
