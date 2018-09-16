#include <ros/ros.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <urdf_parser/urdf_parser.h>

#include <trajopt/problem_description.hpp>
#include <trajectory_msgs/JointTrajectory.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <tesseract_msgs/TesseractState.h>
#include <tesseract_ros/ros_tesseract_utils.h>

#include <geometry_msgs/PoseArray.h>

static bool loadEnvironment(tesseract::tesseract_ros::KDLEnvPtr& env)
{
  ros::NodeHandle nh;

  const static std::string ROBOT_DESCRIPTION ("robot_description");
  const static std::string ROBOT_DESCRIPTION_SEMANTIC ("robot_description_semantic");

  std::string urdf_xml, srdf_xml;
  if (!nh.getParam(ROBOT_DESCRIPTION, urdf_xml)) {
    ROS_WARN("robot_description");
    return false;
  }
  if (!nh.getParam(ROBOT_DESCRIPTION_SEMANTIC, srdf_xml)) {
    ROS_WARN("semantic desc");
    return false;
  }

  auto urdf_model = urdf::parseURDF(urdf_xml);
  if (!urdf_model) {
    ROS_WARN("parse urdf");
    return false;
  }

  auto srdf_model = srdf::ModelSharedPtr(new srdf::Model);
  if (!srdf_model->initString(*urdf_model, srdf_xml)) {
    ROS_WARN("parse srdf");
    return false;
  }

  auto env_ptr = std::make_shared<tesseract::tesseract_ros::KDLEnv>();
  if (env_ptr->init(urdf_model, srdf_model))
  {
    env = env_ptr;
    return true;
  }
  else
  {
    ROS_WARN("!env init");
    return false;
  }
}

static bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
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

static EigenSTL::vector_Isometry3d makePath()
{
  // Define a path in the frame of the positioner
  // The positioner structure is a cube with 0.5 meter sides
  // The frame is located at the bottom center
  EigenSTL::vector_Isometry3d path;
  const Eigen::Isometry3d to_top_surface = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.5);

  // put a circle on the top plane
  const double a= 0.25, b =0.5;
  for (double r = 0; r < 2 * M_PI; r += M_PI / 12)
  {
    const double radius = a * b / (std::sqrt(a * a * std::sin(r) * std::sin(r) + b * b * std::cos(r) * std::cos(r)));
//    const double radius = 0.1;
    Eigen::Isometry3d point = to_top_surface * Eigen::Translation3d(radius * std::cos(r),
                                                                    radius * std::sin(r),
                                                                    0.0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
    path.push_back(point);
  }

  return path;
}

static trajopt::TrajOptProbPtr makeProblem(tesseract::BasicEnvConstPtr env,
                                           const EigenSTL::vector_Isometry3d& geometric_path)
{
  trajopt::ProblemConstructionInfo pci (env);

  // Populate Basic Info
  pci.basic_info.n_steps = geometric_path.size();
  pci.basic_info.manip = "manipulator_positioner";
  pci.basic_info.start_fixed = false;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  const auto dof = pci.kin->numJoints();

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());

  pci.init_info.type = trajopt::InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);
  ROS_ERROR_STREAM("DOF: " << dof);

  // Populate Cost Info
  std::shared_ptr<trajopt::JointVelCostInfo> jv = std::shared_ptr<trajopt::JointVelCostInfo>(new trajopt::JointVelCostInfo);
  jv->coeffs = std::vector<double>(dof, 2.5);
  jv->name = "joint_vel";
  jv->term_type = trajopt::TT_COST;
  pci.cost_infos.push_back(jv);

  std::shared_ptr<trajopt::JointAccCostInfo> ja = std::shared_ptr<trajopt::JointAccCostInfo>(new trajopt::JointAccCostInfo);
  ja->coeffs = std::vector<double>(dof, 2.5);
  ja->name = "joint_acc";
  ja->term_type = trajopt::TT_COST;
  pci.cost_infos.push_back(ja);

  std::shared_ptr<trajopt::CollisionCostInfo> collision = std::shared_ptr<trajopt::CollisionCostInfo>(new trajopt::CollisionCostInfo);
  collision->name = "collision";
  collision->term_type = trajopt::TT_COST;
  collision->continuous = false;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  collision->gap = 1;
  collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 10);

  // Apply a special cost between the sander_disks and the part
  for (auto& c : collision->info)
  {
    c->SetPairSafetyMarginData("welder_tip", "positioner", 0.005, 20.0);
  }

  pci.cost_infos.push_back(collision);

  // Populate Constraints
  for (std::size_t i = 0; i < geometric_path.size(); ++i)
  {
    std::shared_ptr<trajopt::PoseCostInfo> pose = std::shared_ptr<trajopt::PoseCostInfo>(new trajopt::PoseCostInfo);
    pose->term_type = trajopt::TT_CNT;
    pose->name = "waypoint_cart_" + std::to_string(i);
    pose->link = "positioner";
    pose->target = "welder_tcp";
    pose->timestep = i;
    pose->tcp = geometric_path[i];
    pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
    pose->rot_coeffs = Eigen::Vector3d(10, 10, 0);
    pci.cnt_infos.push_back(pose);
  }

  return trajopt::ConstructProblem(pci);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo1");
  ros::NodeHandle nh, pnh ("~");
  ros::AsyncSpinner spinner (1); spinner.start();

  tesseract::tesseract_ros::KDLEnvPtr env;
  if (!loadEnvironment(env))
  {
    return -1;
  }

  // initial conditions?
  auto names = env->getManipulator("manipulator_positioner")->getJointNames();
  env->setState(names, std::vector<double>(names.size(), 0.0));

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("poses", 0, true);

  // visualize
  geometry_msgs::PoseArray poses_msg;
  poses_msg.header.frame_id = "positioner";
  poses_msg.header.stamp = ros::Time::now();

  auto geometric_path = makePath();
  for (const auto& p : geometric_path)
  {
    geometry_msgs::Pose p_msg;
    tf::poseEigenToMsg(p, p_msg);
    poses_msg.poses.push_back(p_msg);
  }

  pub.publish(poses_msg);

  auto opt_problem = makeProblem(env, geometric_path);

  trajopt::BasicTrustRegionSQP optimizer (opt_problem);
  optimizer.initialize(trajopt::trajToDblVec(opt_problem->GetInitTraj()));

  const auto opt_status = optimizer.optimize();
  if (opt_status != trajopt::OptStatus::OPT_CONVERGED) ROS_WARN("Did not converge");

  auto result = trajopt::getTraj(optimizer.x(), opt_problem->GetVars());

  // To & From Vector of parameters
  trajectory_msgs::JointTrajectory out;
  out.joint_names = env->getManipulator("manipulator_positioner")->getJointNames();
  for (const auto& joint : out.joint_names)
  {
    std::cout << "Joint: " << joint << "\n";
  }

  for (int i = 0; i < result.rows(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint jtp;
    for (int j = 0; j < result.cols(); ++j)
    {
      jtp.positions.push_back(result(i, j));
    }
    jtp.time_from_start = ros::Duration(1.0 * i);
    out.points.push_back(jtp);
  }

  tesseract_msgs::TesseractState msg;
  tesseract::tesseract_ros::tesseractToTesseractStateMsg(msg, *env);

  ros::Publisher scene_pub = nh.advertise<tesseract_msgs::TesseractState>("scene", 1, true);


  scene_pub.publish(msg);

  ros::Timer pub_timer = nh.createTimer(ros::Duration(0.1), [&pub, &poses_msg] (const ros::TimerEvent&) {
    poses_msg.header.stamp = ros::Time::now();
    pub.publish(poses_msg);
  }, false, true);

  executeTrajectory(out);

  ros::waitForShutdown();
  return 0;
}
