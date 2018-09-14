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
  EigenSTL::vector_Isometry3d v;
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translation() = Eigen::Vector3d(1.0, 0, 0.75);

  // Create slices of the cylinder
  const static double radius = CYL_RADIUS;
  const static std::size_t n_slices = 1;
  const static double slice_height = 0.1;
  for (std::size_t i = 0; i < n_slices; ++i)
  {
    const double z = i * slice_height;
    const Eigen::Isometry3d slice_center = origin * Eigen::Translation3d(0.0, 0.0, z);
    for (double r = 0.0; r <= 2 * M_PI; r += M_PI / 12.0)
    {
      Eigen::Vector3d offset (radius * std::cos(r), radius * std::sin(r), 0.);
      Eigen::Isometry3d pose = slice_center * Eigen::Translation3d(offset);

      Eigen::Vector3d z_axis = -(pose.translation() - slice_center.translation()).normalized();
      Eigen::Vector3d y_axis = Eigen::Vector3d(-std::sin(r), std::cos(r), 0.0).normalized();
      Eigen::Vector3d x_axis= y_axis.cross(z_axis).normalized();
//      std::cout << "z_axis: " << z_axis.transpose() << "\n";
//      std::cout << "y_axis: " << y_axis.transpose() << "\n";
//      std::cout << "x_axis: " << x_axis.transpose() << "\n";

      pose.matrix().col(2).head<3>() = z_axis;
      pose.matrix().col(1).head<3>() = y_axis;
      pose.matrix().col(0).head<3>() = x_axis;

      v.push_back(pose);
    }
  }

//  const double tilt = -10 * M_PI / 180.;
//  for (auto& p : v)
//  {
//    p = p * Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitX()) * Eigen::Translation3d(0, 0.05, 0);
//  }


  return v;
}

static EigenSTL::vector_Isometry3d makePath2()
{
  EigenSTL::vector_Isometry3d v;
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translation() = Eigen::Vector3d(0.6, 0, 0.5);


  for (int r = 0; r < 5; ++r)
  {
    EigenSTL::vector_Isometry3d this_pass;
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

//  const double tilt = -10 * M_PI / 180.;
//  for (auto& p : v)
//  {
//    p = p * Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitX()) * Eigen::Translation3d(0, 0.05, 0);
//  }


  return v;
}


static std::vector<descartes_light::PositionSamplerPtr> makeSamplers(const EigenSTL::vector_Isometry3d& poses,
                                                                     descartes_light::CollisionInterfacePtr coll_env)
{
  // The current setup requires that our cartesian sampler is aware of the robot
  // kinematics
  opw_kinematics::Parameters<double> kin_params = makeIrb4600_205_60<double>();
  auto tip_to_tool = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.235, 0.0, 0.1) *
                     Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());

  auto world_to_base = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.0, 0.0, 2.75) *
                       Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  descartes_light::RailedKinematicsInterface kin_interface (kin_params, world_to_base, tip_to_tool);

  std::vector<descartes_light::PositionSamplerPtr> result;
  result.reserve(poses.size());

  for (const auto& pose : poses)
  {
    auto collision_clone = descartes_light::CollisionInterfacePtr(coll_env->clone());
    auto sampler = std::make_shared<descartes_light::RailedAxialSymmetricSampler>(pose, kin_interface, M_PI/24.0,
                                                                                  collision_clone);
    result.push_back(std::move(sampler));
  }

  return result;
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

//  addObject(*env);

  if (!env->addManipulator("world_frame", "sander_tcp", "my_robot"))
  {
    ROS_ERROR("Could not create group");
    return -2;
  }

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("poses", 0, true);

  // visualize
  geometry_msgs::PoseArray poses_msg;
  poses_msg.header.frame_id = env->getManipulator("my_robot")->getBaseLinkName();
  poses_msg.header.stamp = ros::Time::now();
  auto path = makePath2();

  for (const auto& p : path)
  {
    geometry_msgs::Pose p_msg;
    tf::poseEigenToMsg(p, p_msg);
    poses_msg.poses.push_back(p_msg);
  }

  pub.publish(poses_msg);


  const auto group_name = "my_robot";
  auto collision_checker = std::make_shared<descartes_light::TesseractCollision>(env, group_name);
  auto samplers = makeSamplers(path, collision_checker);
  auto edge_computer = std::make_shared<descartes_light::DistanceEdgeEvaluator>(std::vector<double>(8, 1.1));
  const auto time_per_point = 0.75;
  auto timing = std::vector<descartes_core::TimingConstraint>(path.size(), time_per_point);

  descartes_light::Solver graph_builder (8);
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
  for (std::size_t i = 0; i < solution.size() / 8; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions.assign(solution.begin() + i * 8, solution.begin() + (i + 1) * 8);
    pt.time_from_start = ros::Duration(i * time_per_point);
    trajectory.points.push_back(pt);
  }

  executeTrajectory(trajectory);

  ros::spin();
  return 0;
}
