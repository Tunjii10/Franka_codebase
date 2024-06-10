// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include <tf/transform_listener.h>

// ROS service
#include "yolov5_ros/petri.h"

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// standard libraries
#include <sstream>
#include <iostream>
#include <cstdlib>

using namespace std;

const double tau = 2 * M_PI;

char proceed;
float x=-0.10; float y = 0.45; float z = 0.55; float r_x=0; float r_y = 4.142; float r_z = 7.10;
  char picking_done;
  char placing_done;
int fruitcount=0;
// function for planner planning and execution (linear movement)
void movement (moveit::planning_interface::MoveGroupInterface& move_group,float x,float y,float z, float x_D,float y_D,float z_D) {
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
  ROS_INFO("Visualizing plan (pose goal) %s", success ? "" : "FAILED");
  cout << target_pose1;
  move_group.execute(my_plan);
  // move_group.move();
  if (!success){
    // if planner not successful shutdown system
    ROS_INFO("Shutting down due to planner failure");
    ros::shutdown();
    }
}



// function to call camera detection service and hand-eye transformation
// function returns detected object in panda link0 frame
vector<float> detect (vector<double> camera_frame) {

  tf::TransformListener listener;
  cout << "camera optical frame: ";
  for (const auto &value : camera_frame) {
        cout << value << " ";
    }
  cout << endl;
  // Wait for the transform from 'panda_link0' to 'camera_color_optical_frame'
  ros::Duration timeout(2.0);
  if (!listener.waitForTransform("panda_link0", "camera_color_optical_frame", ros::Time(0), timeout))
  {
      ROS_ERROR("Transform from 'panda_link0' to 'camera_color_optical_frame' not available.");
      vector<float> error_result(3, 0.0); // Return a vector of zeros
      return error_result;
  }
  // float object_x = camera_frame[0];  // Replace with your object's coordinatesadd_executable(gripper src/gripper.cpp)
  // float object_y = camera_frame[1];
  // float object_z = camera_frame[2];

  float object_x = camera_frame[0]+0.04;  // Replace with your object's coordinatesadd_executable(gripper src/gripper.cpp)
  float object_y = camera_frame[1]-0;
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
      vector<float> error_result(3, 0.0); // Return a vector of zeros
      return error_result;
  }
  vector<float> arr(3);
  // Extract the transformed point in panda_link0 coordinates
  arr[0] = panda_point.point.x-0.035;
  arr[1] = panda_point.point.y-0.02;
  arr[2] = panda_point.point.z;

  return arr;
}



int main(int argc, char* argv[])
{
  // initialize node
  ros::init(argc, argv, "demo");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // initialize publisher and Subscriber
  ros::Publisher line_pub = nh.advertise<std_msgs::String>("/string_zone", 10);


  ros::ServiceClient client = nh.serviceClient<yolov5_ros::petri>("/petri_service");

  ros::Rate loop_rate(10);

  // Create the MoveIt MoveGroup Interface
  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");

  // set velocity and acceleration scaling factors. IMPORTANT!!!
  move_group.setMaxVelocityScalingFactor(0.28);
  move_group.setMaxAccelerationScalingFactor(0.03);

  // planning time
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);

  // iniltialize pilz "LIN" motion planner and set start state
  move_group.setPlannerId("LIN");
  move_group.setStartStateToCurrentState();

  // print start state to console
  // geometry_msgs::PoseStamped current_pose =move_group.getCurrentPose();
  // cout<<current_pose;

  // hard coded waypoints and pose
  // x_D = 0;
  // y_D = 3.142;//3.142;//3.926;
  // z_D = 7.10;//5.49;//0.0;
  // bent - 2.79(add 150's), 1.5 (figuring out), 2.4(add 130s)
  // float home[] = {-0.10, 0.45, 0.8, 0, 3.142, 7.10};
  // float home[] = {-0.10, 0.45, 0.55, 0, 3.142, 7.10}; //franka home position before franka_gripper

  float home[] = {-0.10, 0.45, 0.40, 0, 3.142, 7.10}; //gripper home position
  // count for zonesbreak
  // we use these zo  char picking_done;nes for picking algorithm
  // zone 1 -- topmost stack row , zone 4-- lowest stack row
  int zone = 1;

  // count for enclosurehar picking_done;
  // we use the enclosure count to know which enclosure to place in
  // ***************************************************************************************************************
  int enclosure = 1;
  int correctionX=1,correctionY=1;
// *****************************************************************************************************************
  while (ros::ok()){
    fruitcount+=1;
    // ********home ***********
    x = 0.25;y=0.1;z=home[2];r_x=home[3];r_y=home[4];r_z=home[5];
    //movement(move_group,x,y,z,r_x,r_y,r_z);

    // movement(move_group, home[0], home[1], home[2], home[3], home[4], home[5]);
    // movement(move_group, home[0], home[1], 0.23, 0, 3.142, 7.2);

    //Test
    // while(true){
    //   movemen  char picking_done;t(move_group,x,y,z,r_x,r_y,r_z);
    //   cout<<"Enter Y to proceed or N to repeat movement";
    //   cin.get(proceed);
    //   if(proceed != 'Y'){
    //     cout<<"Enter new x";
    //     cin>>x;
    //
    //   }
    //   else{
    //     break;
    //   }
    //
    // }

    // %%%%%go to home position for camera detection
    movement(move_group, home[0], home[1], home[2], home[3], home[4], home[5]);
    // jointMove(move_group);
    ros::Duration(1).sleep();

    // service proxy request initialize
    yolov5_ros::petri srv;

    // publish line to pick from for camera detection
    std_msgs::String msg;
    stringstream ss;
    ss << "depth estimation zone"<<" "<<zone;
    msg.data = ss.str();
    line_pub.publish(msg);
    ros::Duration(3).sleep(); // sleep for center point to be detected properly

    // service request and call ros::Duration(1).sleep();- camera detection
    int detect_service_var = 1;
    srv.request.detect = detect_service_var;
    vector<float> transformedPointsPanda;  char picking_done;

    if (client.call(srv))
    // **********pick descent operation *************
    {

      ROS_INFO("Object in camera optical frame called");
      vector<double> transformedPoints(srv.response.result.begin(), srv.response.result.end());
      transformedPointsPanda = detect(transformedPoints);
      ROS_INFO("Object in pandalink0 frame found");
      for (const auto &value : transformedPointsPanda) {
            cout << value << " ";
        }
      cout << endl;

      // // publish ready for picking
      //
      // ros::Duration(1).sleep();
      // // %%%%%% go to detected object position - directly above
      // movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.49, 2.76, 0.35, 3.6);//0.4
      //
      // ros::Duration(1).sleep();
      // for zone 1
      // %%%%%%%%% go to pick descent operation 1
      //movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.153, 0, 3.142, 7);//0.1
      //movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.41, 0, 3.142, 7.2);//0.1
      correctionX=1.1;
        correctionY=1.1;
      movement(move_group, transformedPointsPanda[0]*correctionX, transformedPointsPanda[1]*correctionY, 0.33, 0, 3.142, 7.2);//0.1
      movement(move_group, transformedPointsPanda[0]*correctionX, transformedPointsPanda[1]*correctionY, 0.23, 0, 3.142, 7.2);//0.1


      ros::Duration(3).sleep();

      cout<<"\n<";
      cout<<"Press a key";
      cin.get(picking_done);
      //cin.get(picking_done);
      ros::Duration(1).sleep();
      // %%%%%go to home position for drop location
      movement(move_group, home[0], home[1], home[2], home[3], home[4], home[5]);

      if(fruitcount==1){
        movement(move_group, 0.2,0.45, 0.35, 0, 3.142, 7.10);
        movement(move_group, 0.2,0.45, 0.25, 0, 3.142, 7.10);
      }
      else if(fruitcount==2){
        movement(move_group, 0.2,0.55, 0.35, 0, 3.142, 7.10);
        movement(move_group, 0.2,0.55, 0.25, 0, 3.142, 7.10);
      }
      else if(fruitcount==3){
        movement(move_group, 0.2,0.65, 0.35, 0, 3.142, 7.10);
        movement(move_group, 0.2,0.65, 0.25, 0, 3.142, 7.10);
      }
      else if(fruitcount==4){
        movement(move_group, 0.1,0.45, 0.35, 0, 3.142, 7.10);
        movement(move_group, 0.1,0.45, 0.25, 0, 3.142, 7.10);
      }

      else if(fruitcount==5){
        movement(move_group, 0.1,0.55, 0.35, 0, 3.142, 7.10);
        movement(move_group, 0.1,0.55, 0.25, 0, 3.142, 7.10);
      }
      else if(fruitcount==6){
        movement(move_group, 0.1,0.65, 0.35, 0, 3.142, 7.10);
        movement(move_group, 0.1,0.65, 0.25, 0, 3.142, 7.10);
      }
      else if(fruitcount==7){
        movement(move_group, 0.0,0.45, 0.35, 0, 3.142, 7.10);
        movement(move_group, 0.0,0.45, 0.25, 0, 3.142, 7.10);
      }
      else if(fruitcount==8){
        movement(move_group, 0.0,0.55, 0.35, 0, 3.142, 7.10);
        movement(move_group, 0.0,0.55, 0.25, 0, 3.142, 7.10);
      }
      else {
        movement(move_group, 0.0,0.65, 0.35, 0, 3.142, 7.10);
        movement(move_group, 0.0,0.65, 0.25, 0, 3.142, 7.10);
      }



      ros::Duration(3).sleep();

      cout<<"\n<";
      cout<<"Press a key";
      ros::Duration(1).sleep();
      cin.get(placing_done);
      ros::Duration(3).sleep();

      cout<<"\n<";
      cout<<"Press a key";
      cin.get(placing_done);
      //cin.get(picking_done);
      ros::Duration(1).sleep();
      movement(move_group, 0.2,0.45, 0.35, 0, 3.142, 7.10);

    }

  ros::spinOnce();
  }
  return 0;
}
