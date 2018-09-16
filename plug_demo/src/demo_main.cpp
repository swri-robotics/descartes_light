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

struct Path
{
  Eigen::Isometry3d mr_pose;
  std::vector<Eigen::Isometry3d> wr_poses;
};

static Path makePath()
{
  Path path;
  path.mr_pose = Eigen::Isometry3d::Identity() * Eigen::Translation3d(1.5, 0.75, 1.0) *
                 Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
  auto wr_pose = Eigen::Isometry3d::Identity() * Eigen::Translation3d(1.5, 0.9, 1.0) *
                 Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());

//  for (int i = -5; i < 5; ++i)
//  {
//    path.wr_poses.push_back(
//          wr_pose * Eigen::Translation3d(i * 0.02, 0, 0)
//        );
//  }

  double radius = 0.15;

  for (double r = 0; r < M_PI * 2.0; r += M_PI / 12)
  {
    auto p = path.mr_pose * Eigen::Translation3d(radius * std::cos(r),
                                                 radius * std::sin(r),
                                                 0.0);
    path.wr_poses.push_back(p);
  }

  return path;
}

static trajopt::TrajOptProbPtr makeProblem(tesseract::BasicEnvConstPtr env,
                                           const Path& path)
{
  trajopt::ProblemConstructionInfo pci (env);

  // Populate Basic Info
  pci.basic_info.n_steps = path.wr_poses.size();
  pci.basic_info.manip = "all";
  pci.basic_info.start_fixed = false;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  const auto dof = pci.kin->numJoints();

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());

  pci.init_info.type = trajopt::InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);
  for (int i = 0; i < pci.basic_info.n_steps; ++i)
  {
    double ratio = static_cast<double>(i + 1) / pci.basic_info.n_steps;
    pci.init_info.data(i, 13) = (1.0 - ratio) * 2.0;
  }

  ROS_ERROR_STREAM("DOF: " << dof);

  // Populate Cost Info
  std::shared_ptr<trajopt::JointVelCostInfo> jv = std::shared_ptr<trajopt::JointVelCostInfo>(new trajopt::JointVelCostInfo);
  jv->coeffs = std::vector<double>(dof, 1.0);
  jv->name = "joint_vel";
  jv->term_type = trajopt::TT_COST;
  pci.cost_infos.push_back(jv);

//  std::shared_ptr<trajopt::JointAccCostInfo> ja = std::shared_ptr<trajopt::JointAccCostInfo>(new trajopt::JointAccCostInfo);
//  ja->coeffs = std::vector<double>(dof, 1.0);
//  ja->name = "joint_acc";
//  ja->term_type = trajopt::TT_COST;
//  pci.cost_infos.push_back(ja);

  std::shared_ptr<trajopt::CollisionCostInfo> collision = std::shared_ptr<trajopt::CollisionCostInfo>(new trajopt::CollisionCostInfo);
  collision->name = "collision";
  collision->term_type = trajopt::TT_COST;
  collision->continuous = false;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  collision->gap = 0;
  collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.02, 20);

  pci.cost_infos.push_back(collision);

  auto to_wxyz = [](const Eigen::Isometry3d& p) {
    Eigen::Quaterniond q (p.linear());
    Eigen::Vector4d wxyz;
    wxyz(0) = q.w();
    wxyz(1) = q.x();
    wxyz(2) = q.y();
    wxyz(3) = q.z();
    return wxyz;
  };

  for (int i = 0; i < pci.basic_info.n_steps; ++i)
  {
    auto pose = std::shared_ptr<trajopt::StaticPoseCostInfo>(new trajopt::StaticPoseCostInfo);
    pose->term_type = trajopt::TT_CNT;
    pose->name = "wr_wp_cart_" + std::to_string(i);
    pose->link = "welder_tcp";
    pose->timestep = i;
    pose->xyz = path.wr_poses[i].translation();
    pose->wxyz = to_wxyz(path.wr_poses[i]);
    pose->pos_coeffs = Eigen::Vector3d(20, 20, 20);
    pose->rot_coeffs = Eigen::Vector3d(0, 0, 0);
    pci.cnt_infos.push_back(pose);

    auto mr_pose = std::shared_ptr<trajopt::StaticPoseCostInfo>(new trajopt::StaticPoseCostInfo);
    mr_pose->term_type = trajopt::TT_CNT;
    mr_pose->name = "mr_wp_cart_" + std::to_string(i);
    mr_pose->link = "mr_tool0";
    mr_pose->timestep = i;
    mr_pose->xyz = path.mr_pose.translation();
    mr_pose->wxyz = to_wxyz(path.mr_pose);
    mr_pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
    mr_pose->rot_coeffs = Eigen::Vector3d(10, 10, 10);
    pci.cnt_infos.push_back(mr_pose);
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
  auto names = env->getManipulator("all")->getJointNames();
  env->setState(names, std::vector<double>(names.size(), 0.0));
  env->setState({"wr_track_rail"}, {2.0});

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("poses", 0, true);

  // visualize
  geometry_msgs::PoseArray poses_msg;
  poses_msg.header.frame_id = "world_frame";
  poses_msg.header.stamp = ros::Time::now();

  auto geometric_path = makePath();
  {
    geometry_msgs::Pose p_msg;
    tf::poseEigenToMsg(geometric_path.mr_pose, p_msg);
    poses_msg.poses.push_back(p_msg);
  }
  for (const auto& p : geometric_path.wr_poses)
  {
    geometry_msgs::Pose p_msg;
    tf::poseEigenToMsg(p, p_msg);
    poses_msg.poses.push_back(p_msg);
  }
  pub.publish(poses_msg);

  auto opt_problem = makeProblem(env, geometric_path);

  trajopt::BasicTrustRegionSQP optimizer (opt_problem);
  optimizer.initialize(trajopt::trajToDblVec(opt_problem->GetInitTraj()));
  auto params = optimizer.getParameters();
  params.max_iter = 1000;
  params.max_merit_coeff_increases = 10;
  optimizer.setParameters(params);

  const auto opt_status = optimizer.optimize();
  if (opt_status != trajopt::OptStatus::OPT_CONVERGED) ROS_WARN("Did not converge");

  auto result = trajopt::getTraj(optimizer.x(), opt_problem->GetVars());

  // To & From Vector of parameters
  trajectory_msgs::JointTrajectory out;
  out.joint_names = env->getManipulator("all")->getJointNames();
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

  tesseract::ContactResultMap map;
  env->getDiscreteContactManager()->contactTest(map);
  for (auto& p : map)
  {
    std::cout << p.first.first << " - " << p.first.second << "\n";
    std::cout << "Is coll allowed? : "<< env->getAllowedCollisionMatrix()->isCollisionAllowed(p.first.first, p.first.second) << "\n";
  }



  scene_pub.publish(msg);

  ros::Timer pub_timer = nh.createTimer(ros::Duration(0.1), [&pub, &poses_msg] (const ros::TimerEvent&) {
    poses_msg.header.stamp = ros::Time::now();
    pub.publish(poses_msg);
  }, false, true);

  executeTrajectory(out);

  ros::waitForShutdown();
  return 0;
}
