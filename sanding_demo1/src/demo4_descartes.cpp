#include <ros/ros.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <urdf_parser/urdf_parser.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <tesseract_msgs/TesseractState.h>
#include <tesseract_ros/ros_tesseract_utils.h>

#include <geometry_msgs/PoseArray.h>

#include <descartes_light/descartes_light.h>
#include <opw_kinematics/opw_parameters_examples.h>
#include <descartes_light/external_axis_sampler.h>

template <typename T>
opw_kinematics::Parameters<T> makeIrb4600_205_60()
{
  opw_kinematics::Parameters<T> p;
  p.a1 = T(0.175);
  p.a2 = T(-0.175);
  p.b =  T(0.000);
  p.c1 = T(0.495);
  p.c2 = T(0.900);
  p.c3 = T(0.960);
  p.c4 = T(0.135);

  p.offsets[2] = -M_PI / 2.0;

  return p;
}

const static double CYL_RADIUS = 0.2;

static bool addObject(tesseract::tesseract_ros::KDLEnv& env)
{
  auto obj = std::make_shared<tesseract::AttachableObject>();

  const static double radius = CYL_RADIUS;
  const static double length = 1.0;
  auto shape = std::make_shared<shapes::Cylinder>(radius, length);

  obj->name = "part";
  obj->visual.shapes.push_back(shape);
  obj->visual.shape_poses.push_back(Eigen::Isometry3d::Identity());
  obj->collision.shapes.push_back(shape);
  obj->collision.shape_poses.push_back(Eigen::Isometry3d::Identity());
  obj->collision.collision_object_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  // This call adds the object to the scene's "database" but does not actuall connect it
  env.addAttachableObject(obj);

  // To include the object in collision checks, you have to attach it
  tesseract::AttachedBodyInfo attached_body;
  attached_body.object_name = "part";
  attached_body.parent_link_name = "world_frame";
  attached_body.transform.setIdentity();
  attached_body.transform.translate(Eigen::Vector3d(1.0, 0, 0.5));

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

static EigenSTL::vector_Isometry3d makePath()
{
  // Define a path in the frame of the positioner
  // The positioner structure is a cylinder of radius 0.25 meters
  EigenSTL::vector_Isometry3d path;

  // put a circle on the top plane
  double s = -0.25;
  for (double r = 0.0; r < 4 * M_PI; r += M_PI / 12)
  {
    Eigen::Isometry3d point = Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitZ()) *
                              Eigen::Translation3d(0, 0.26, s) * Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    s += 0.015;
    path.push_back(point);
  }

  return path;
}

static std::vector<descartes_light::PositionSamplerPtr> makeSamplers(const EigenSTL::vector_Isometry3d& poses,
                                                                     descartes_light::CollisionInterfacePtr coll_env)
{
  // The current setup requires that our cartesian sampler is aware of the robot
  // kinematics
  opw_kinematics::Parameters<double> kin_params = makeIrb4600_205_60<double>();
  const auto tip_to_tool = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.06, 0.0, 0.2125) *
                           Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitY());
  const auto world_to_base = Eigen::Isometry3d::Identity();

  descartes_light::KinematicsInterface kin_interface (kin_params, world_to_base, tip_to_tool);

  std::vector<descartes_light::PositionSamplerPtr> result;
  result.reserve(poses.size());

  for (const auto& pose : poses)
  {
    auto collision_clone = descartes_light::CollisionInterfacePtr(coll_env->clone());
    auto sampler = std::make_shared<descartes_light::SpoolSampler>(pose, kin_interface, collision_clone);
    result.push_back(std::move(sampler));
  }

  return result;
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

//  addObject(*env);
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("poses", 0, true);

  // visualize
  geometry_msgs::PoseArray poses_msg;
  poses_msg.header.frame_id = "positioner";
  poses_msg.header.stamp = ros::Time::now();
  auto path = makePath();

  for (const auto& p : path)
  {
    geometry_msgs::Pose p_msg;
    tf::poseEigenToMsg(p, p_msg);
    poses_msg.poses.push_back(p_msg);
  }

  pub.publish(poses_msg);


  const auto group_name = "manipulator_positioner";
  auto collision_checker = std::make_shared<descartes_light::TesseractCollision>(env, group_name);
  auto samplers = makeSamplers(path, collision_checker);
  auto edge_computer = std::make_shared<descartes_light::DistanceEdgeEvaluator>(std::vector<double>(7, 1.1));
  const auto time_per_point = 0.75;
  auto timing = std::vector<descartes_core::TimingConstraint>(path.size(), time_per_point);

  descartes_light::Solver graph_builder (7);
  if (!graph_builder.build(samplers, timing, edge_computer))
  {
    std::cerr << "Failed to build graph\n";
    return 1;
  }

  // Search for edges
  std::vector<double> solution;
  if (!graph_builder.search(solution))
  {
    std::cerr << "Search for graph completion failed\n";
    return 1;
  }

  // To joint trajectory
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = env->getManipulator(group_name)->getJointNames();
  for (const auto& name : trajectory.joint_names)
  {
    std::cout << "Joint name: " << name << "\n";
  }
  for (std::size_t i = 0; i < solution.size() /7; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions.assign(solution.begin() + i * 7, solution.begin() + (i + 1) * 7);
    pt.time_from_start = ros::Duration(i * time_per_point);
    trajectory.points.push_back(pt);
  }

  ros::Timer pub_timer = nh.createTimer(ros::Duration(0.1), [&pub, &poses_msg] (const ros::TimerEvent&) {
    poses_msg.header.stamp = ros::Time::now();
    pub.publish(poses_msg);
  }, false, true);

  executeTrajectory(trajectory);

  ros::waitForShutdown();
  return 0;
}
