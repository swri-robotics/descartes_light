#include <ros/ros.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <urdf_parser/urdf_parser.h>

#include <trajopt/problem_description.hpp>
#include <trajectory_msgs/JointTrajectory.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <tesseract_msgs/TesseractState.h>
#include <tesseract_ros/ros_tesseract_utils.h>

static bool addObject(tesseract::tesseract_ros::KDLEnv& env)
{
  auto obj = std::make_shared<tesseract::AttachableObject>();

  auto box = std::make_shared<shapes::Box>(1.0, 1.0, 1.0);

  obj->name = "part";
  obj->visual.shapes.push_back(box);
  obj->visual.shape_poses.push_back(Eigen::Affine3d::Identity());
  obj->collision.shapes.push_back(box);
  obj->collision.shape_poses.push_back(Eigen::Affine3d::Identity());
  obj->collision.collision_object_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  // This call adds the object to the scene's "database" but does not actuall connect it
  env.addAttachableObject(obj);

  // To include the object in collision checks, you have to attach it
  tesseract::AttachedBodyInfo attached_body;
  attached_body.object_name = "part";
  attached_body.parent_link_name = "world_frame";
  attached_body.transform.setIdentity();
  attached_body.transform.translate(Eigen::Vector3d(1.0, 0, 0));

  env.attachBody(attached_body);
  return true;
}

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

static EigenSTL::vector_Affine3d makePath()
{
  EigenSTL::vector_Affine3d v;
  Eigen::Affine3d origin = Eigen::Affine3d::Identity();
  origin.translation() = Eigen::Vector3d(0.5, 0, 0.5);


  for (int r = 0; r < 10; ++r)
  {
    EigenSTL::vector_Affine3d this_pass;
    for (int i = -10; i <= 10; ++i)
    {
      auto p = origin * Eigen::Translation3d(r * 0.1, i * 0.05, 0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
      this_pass.push_back(p);
    }

    if (r % 2 != 0)
    {
      std::reverse(this_pass.begin(), this_pass.end());
      for (auto& p : this_pass)
        p = p * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    }

    v.insert(v.end(), this_pass.begin(), this_pass.end());
  }

  const double tilt = -10 * M_PI / 180.;
  for (auto& p : v)
  {
    p = p * Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitX()) * Eigen::Translation3d(0, 0.05, 0);
  }


  return v;
}

static EigenSTL::vector_Affine3d makePath2()
{
  EigenSTL::vector_Affine3d v;
  Eigen::Affine3d origin = Eigen::Affine3d::Identity();
  origin.translation() = Eigen::Vector3d(1.0, 0, 0.5);

  double angle_step = M_PI * 2 / 30;
  const double r = 0.25;

  for (double s = 0; s < 2 * M_PI; s += angle_step)
  {
    auto p = origin * Eigen::Translation3d(r*std::cos(s), r*std::sin(s), 0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-s, Eigen::Vector3d::UnitZ());
    v.push_back(p);
  }

  const double tilt = -20 * M_PI / 180.;
  for (auto& p : v)
  {
    p = p * Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitX()) * Eigen::Translation3d(0, 0.05, 0);
  }


  return v;
}

static trajopt::TrajOptProbPtr makeProblem(tesseract::BasicEnvConstPtr env)
{
  auto geometric_path = makePath();
  trajopt::ProblemConstructionInfo pci (env);

  // Populate Basic Info
  pci.basic_info.n_steps = geometric_path.size();
  pci.basic_info.manip = "my_robot";
  pci.basic_info.start_fixed = false;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  const auto dof = pci.kin->numJoints();

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());

  pci.init_info.type = trajopt::InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Populate Cost Info
  std::shared_ptr<trajopt::JointVelCostInfo> jv = std::shared_ptr<trajopt::JointVelCostInfo>(new trajopt::JointVelCostInfo);
  jv->coeffs = std::vector<double>(dof, 2.5);
  jv->name = "joint_vel";
  jv->term_type = trajopt::TT_COST;
  pci.cost_infos.push_back(jv);

  std::shared_ptr<trajopt::JointAccCostInfo> ja = std::shared_ptr<trajopt::JointAccCostInfo>(new trajopt::JointAccCostInfo);
  ja->coeffs = std::vector<double>(dof, 5.0);
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
  collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 20);

  // Apply a special cost between the sander_disks and the part
  for (auto& c : collision->info)
  {
    c->SetPairSafetyMarginData("sander_disk", "part", -0.01, 20.0);
    c->SetPairSafetyMarginData("sander_shaft", "part", 0.0, 20.0);
  }

  pci.cost_infos.push_back(collision);

  auto to_wxyz = [](const Eigen::Affine3d& p) {
    Eigen::Quaterniond q (p.linear());
    Eigen::Vector4d wxyz;
    wxyz(0) = q.w();
    wxyz(1) = q.x();
    wxyz(2) = q.y();
    wxyz(3) = q.z();
    return wxyz;
  };

  // Populate Constraints
  for (std::size_t i = 0; i < geometric_path.size(); ++i)
  {
    std::shared_ptr<trajopt::StaticPoseCostInfo> pose = std::shared_ptr<trajopt::StaticPoseCostInfo>(new trajopt::StaticPoseCostInfo);
    pose->term_type = trajopt::TT_CNT;
    pose->name = "waypoint_cart_" + std::to_string(i);
    pose->link = "sander_tcp";
    pose->timestep = i;
    pose->xyz = geometric_path[i].translation();
    pose->wxyz = to_wxyz(geometric_path[i]);
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

  tesseract::tesseract_ros::KDLEnvPtr env;
  if (!loadEnvironment(env))
  {
    return -1;
  }

  addObject(*env);

  if (!env->addManipulator("world_frame", "sander_tcp", "my_robot"))
  {
    ROS_ERROR("Could not create group");
    return -2;
  }

  // initial conditions?
  auto names = env->getJointNames();
  env->setState(names, std::vector<double>(names.size(), 0.0));

  std::vector<std::string> start_state_names = {"joint_6"};
  std::vector<double> start_state_values = {-M_PI};
  env->setState(start_state_names, start_state_values);

  auto opt_problem = makeProblem(env);

  trajopt::BasicTrustRegionSQP optimizer (opt_problem);
  optimizer.initialize(trajopt::trajToDblVec(opt_problem->GetInitTraj()));

  const auto opt_status = optimizer.optimize();
  if (opt_status != trajopt::OptStatus::OPT_CONVERGED) ROS_WARN("Did not converge");

  auto result = trajopt::getTraj(optimizer.x(), opt_problem->GetVars());

  // To & From Vector of parameters
  trajectory_msgs::JointTrajectory out;
  out.joint_names = env->getManipulator("my_robot")->getJointNames();
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


  executeTrajectory(out);

  ros::spin();
  return 0;
}
