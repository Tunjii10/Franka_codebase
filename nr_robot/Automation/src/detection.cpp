#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <iostream>
#include <vector>

#include "yolov5_ros/petri.h"
#include <cstdlib>

// void interpolate (){
//   ros::init(argc, argv, "pose_publisher");
//   ros::NodeHandle nh;
//   ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_topic", 1);

//   geometry_msgs::PoseStamped start_pose;
//   // Initialize start_pose and other variables as needed

//   ros::Time start_time = ros::Time::now();

//   double speed = 1.0;  // Set your desired speed
//   int num_steps = static_cast<int>(speed * 10);
//   double step_size = 1.0 / num_steps;

//   tf::Quaternion orient;
//   if (orient.size() == 3)
//   {
//       tf::Matrix3x3 rotation_matrix;
//       rotation_matrix.setEulerYPR(orient[0], orient[1], orient[2]);
//       tf::Quaternion goal_quat;
//       rotation_matrix.getRotation(goal_quat);
//       orient = goal_quat;
//   }

//   tf::Quaternion start_orient(start_pose.pose.orientation.x, start_pose.pose.orientation.y,
//                               start_pose.pose.orientation.z, start_pose.pose.orientation.w);

//   tf::Quaternion orient_new = orient * start_orient;

//   double old_x = 0.0;
//   double old_y = 0.0;
//   double old_z = 0.0;

//   ros::Rate rate(10);  // Publish at 10 Hz

//   for (int i = 0; i < num_steps; ++i)
//   {
//       double t = static_cast<double>(i) / (num_steps - 1);
//       geometry_msgs::PoseStamped pose_to_pub;

//       pose_to_pub.pose.position.x = (1.0 - t) * start_pose.pose.position.x + t * pos[0];
//       pose_to_pub.pose.position.y = (1.0 - t) * start_pose.pose.position.y + t * pos[1];
//       pose_to_pub.pose.position.z = (1.0 - t) * start_pose.pose.position.z + t * pos[2];

//       pose_to_pub.pose.orientation.x = orient_new.x();
//       pose_to_pub.pose.orientation.y = orient_new.y();
//       pose_to_pub.pose.orientation.z = orient_new.z();
//       pose_to_pub.pose.orientation.w = orient_new.w();

//       pose_to_pub.header.frame_id = "panda_link0";
//       pose_to_pub.header.stamp = ros::Time::now();
//       pose_pub.publish(pose_to_pub);

//       ROS_INFO("----pose published-----");
//       ROS_INFO_STREAM(pose_to_pub);
//       ROS_INFO("----end pose published-----");

//       if (pose_to_pub.pose.position.x == old_x && pose_to_pub.pose.position.y == old_y &&
//           pose_to_pub.pose.position.z == old_z)
//       {
//           ROS_INFO("stopped");
//           break;
//       }

//       old_x = pose_to_pub.pose.position.x;
//       old_y = pose_to_pub.pose.position.y;
//       old_z = pose_to_pub.pose.position.z;

//       rate.sleep();
//   }

// }

void movement (moveit::planning_interface::MoveGroupInterface& move_group,float& x,float& y,float& z, float& x_D,float& y_D,float& z_D) {
  geometry_msgs::Pose target_pose1;
  tf2::Quaternion orientation;
  orientation.setRPY(x_D, y_D, z_D);
  target_pose1.orientation = tf2::toMsg(orientation);
  // target_pose1.orientation.w = 0;
  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;
  move_group.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  std::cout << target_pose1;
  move_group.execute(my_plan);
  // move_group.move();
}

std::vector<float> detect (std::vector<double> camera_frame) {
  tf::TransformListener listener;
  std::cout << "camera optical frame: ";
  for (const auto &value : camera_frame) {
        std::cout << value << " ";
    }
  std::cout << std::endl;
  // Wait for the transform from 'panda_link0' to 'camera_color_optical_frame'
  ros::Duration timeout(2.0);
  if (!listener.waitForTransform("panda_link0", "camera_color_optical_frame", ros::Time(0), timeout))
  {
      ROS_ERROR("Transform from 'panda_link0' to 'camera_color_optical_frame' not available.");
      std::vector<float> error_result(3, 0.0); // Return a vector of zeros
      return error_result;
  }
  float object_x = camera_frame[0];  // Replace with your object's coordinates
  float object_y = camera_frame[1];
  float object_z = camera_frame[2];

  // Create a PointStamped message to represent the object's position
  geometry_msgs::PointStamped camera_point;
  camera_point.header.frame_id = "camera_color_optical_frame";
  camera_point.header.stamp = ros::Time(0);
  camera_point.point.x = object_x;
  camera_point.point.y = object_y;
  camera_point.point.z = object_z;

  geometry_msgs::PointStamped panda_point;

  try
  {
      listener.transformPoint("panda_link0", camera_point, panda_point);
  }
  catch (tf::TransformException& ex)
  {
      ROS_ERROR("Failed to transform point: %s", ex.what());
      std::vector<float> error_result(3, 0.0); // Return a vector of zeros
      return error_result;
  }
  std::vector<float> arr(3);
  // Extract the transformed point in panda_link0 coordinates
  arr[0] = panda_point.point.x;
  arr[1] = panda_point.point.y;
  arr[2] = panda_point.point.z;
  // You can now use the 'object_x_panda', 'object_y_panda', and 'object_z_panda' variables
  // as the object's coordinates in panda_link0 coordinates.

  return arr;
}


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.6;
  collision_objects[0].primitives[0].dimensions[2] = 1.2;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = -0.6;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.6;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 1.2;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = -0.4;
  collision_objects[1].primitive_poses[0].position.z = 0.6;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[2].id = "table3";
  collision_objects[2].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 1.4;
  collision_objects[2].primitives[0].dimensions[1] = 0.4;
  collision_objects[2].primitives[0].dimensions[2] = 0.6;

  /* Define the pose of the table. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.7;
  collision_objects[2].primitive_poses[0].position.y = 0.6;
  collision_objects[2].primitive_poses[0].position.z = 0.3;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;


  planning_scene_interface.applyCollisionObjects(collision_objects);
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "detection");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::ServiceClient client = node_handle.serviceClient<yolov5_ros::petri>("/petri_service");
  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.05);

  // addCollisionObjects(planning_scene_interface);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
  
  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link0");
  
  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  for (int i = 0; i < 3; i++){
    // home
    float x,y,z, x_D, y_D, z_D;
    // x = 0.25;
    // y = 0.0;
    // z = 0.8;
    x = 0.3;
    y = 0.0;
    z = 0.8;
    x_D = 0;
    y_D = 3.926;//3.142;
    z_D = 0;//5.49;
    movement(move_group, x, y, z, x_D, y_D, z_D);

    // // transversals operations
    // ros::Duration(2).sleep(); // sleep for camera detection
    // yolov5_ros::petri srv;
    // int detect_var = 1;
    // srv.request.detect = detect_var;
    // if (client.call(srv))
    // {
    //   // pick above operation
    //   ROS_INFO("Object in camera optical frame called");
    //   std::vector<double> transformedPoints(srv.response.result.begin(), srv.response.result.end());
    //   std::vector<float> transformedPointsPanda = detect(transformedPoints);
    //   ROS_INFO("Object in pandalink0 frame found");
    //   for (const auto &value : transformedPointsPanda) {
    //         std::cout << value << " ";
    //     }
    //   std::cout << std::endl;
    //   float x = transformedPointsPanda[0];
    //   float y = transformedPointsPanda[1];
    //   float z = 0.7;//transformedPointsPanda[2];
    //   movement(move_group, x, y, z, x_D, y_D, z_D);

    //   // pick operation 1
    //   z = 0.61;
    //   movement(move_group, x, y, z, x_D, y_D, z_D);

    //   // pick operation 2
    //   z = 0.31;
    //   movement(move_group, x, y, z, x_D, y_D, z_D);

    //   // pick operation 3
    //   z = 0.28;//0.26;
    //   movement(move_group, x, y, z, x_D, y_D, z_D);
    //   ros::Duration(2).sleep();

    //   // pick rise operation 1
    //   z = 0.40;
    //   movement(move_group, x, y, z, x_D, y_D, z_D);

    //   // pick rise operation 2
    //   z = 0.65;
    //   movement(move_group, x, y, z, x_D, y_D, z_D);

    //   // pick rise operation 3
    //   z = 0.80;
    //   movement(move_group, x, y, z, x_D, y_D, z_D);

    //   // place midpoint operation 1
    //   y = 0.37;
    //   movement(move_group, x, y, z, x_D, y_D, z_D);

    //   // place midpoint operation 2
    //   x = -0.19;
    //   movement(move_group, x, y, z, x_D, y_D, z_D);

    //   // place directly above operation
    //   x = -0.214;
    //   y = 0.4663;
    //   z = 0.80;
    //   movement(move_group, x, y, z, x_D, y_D, z_D);

    //   // place drop 1 operation
    //   x = -0.214;
    //   y = 0.4663;
    //   z = 0.62;
    //   movement(move_group, x, y, z, x_D, y_D, z_D);

    //   // place drop 2 operation
    //   x = -0.214;
    //   y = 0.4663;
    //   z = 0.42;
    //   movement(move_group, x, y, z, x_D, y_D, z_D);
    // } else {
    //   ROS_ERROR("failed to call service");
    //   return 1;
    // }

  }




  ros::waitForShutdown();
  return 0;
}
