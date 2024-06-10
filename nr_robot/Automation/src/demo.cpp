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

bool systemReady = true;
bool startRemovingBags = true;
bool resetDone = false;
bool pickFailed = false;
bool pickSuccess = false;
bool bottomSuctionOn = false;
int positionCorrect = 0;
int postionIncorrect = 0;
bool bottomSuctionSuccess = false;
bool bottomSuctionFailed = false;

// systemReady callback
void systemReadyCallback(const std_msgs::Int32::ConstPtr& msg){
  if (msg->data == 1){
    systemReady = true;
  }
}

// reset done callback
void resetDoneCallback (const std_msgs::Int32::ConstPtr& msg){
  if (msg->data == 1){
    resetDone = true;
  }
}

// pick failed callback
void pickingFailedCallback (const std_msgs::Bool::ConstPtr& msg){
  if (msg->data == true){
    pickFailed = true;
  }
}

// pick success callback
void pickingSuccessCallback (const std_msgs::Bool::ConstPtr& msg){
  if (msg->data == true){
    pickSuccess = true;
  }
}

// bottom suction callback
void bottomSuctionOnCallback (const std_msgs::Bool::ConstPtr& msg){
  if (msg->data == true){
    bottomSuctionOn = true;
  }
}

// position correct callback
void positionCorrectCallback (const std_msgs::Int32::ConstPtr& msg){
  if (msg->data == 1){
    positionCorrect = 1;
  }
}

// position incorrect callback
void positionInCorrectCallback (const std_msgs::Int32::ConstPtr& msg){
  if (msg->data == 1){
    postionIncorrect = 1;
  }
}

// bottom suction success callback
void bottomSuctionSuccessCallback (const std_msgs::Bool::ConstPtr& msg){
  if (msg->data == true){
    bottomSuctionSuccess = true;
  }
}

// bottom suction failed callback
void bottomSuctionFailedCallback (const std_msgs::Bool::ConstPtr& msg){
  if (msg->data == true){
    bottomSuctionFailed = true;
  }
}

void startRemovingBagsCallback (const std_msgs::Int32::ConstPtr& msg){
  if (msg->data == 1){
    startRemovingBags = true;
  }
}


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

  ros::Publisher ready_picking_pub = nh.advertise<std_msgs::Int32>("/ready_for_picking", 10);

  ros::Publisher over_drop_position_pub = nh.advertise<std_msgs::Int32>("/over_drop_position", 10);

  ros::Publisher finished_dropping_cycle_pub = nh.advertise<std_msgs::Int32>("/finished_dropping_cycle", 10);

  ros::Publisher removing_bag_status_check_pub = nh.advertise<std_msgs::Int32>("/removing_bag_status_check", 10);


  ros::Subscriber system_ready = nh.subscribe("/system_ready", 10, systemReadyCallback);

  ros::Subscriber reset_done = nh.subscribe("/reset_done", 10, resetDoneCallback);

  ros::Subscriber picking_failed = nh.subscribe("/picking_failed", 10, pickingFailedCallback);

  ros::Subscriber picking_success = nh.subscribe("/picking_success", 10, pickingSuccessCallback);

  ros::Subscriber bottom_suction = nh.subscribe("/bottom_suction_on", 10, bottomSuctionOnCallback);

  ros::Subscriber position_correct = nh.subscribe("/position_correct", 10, positionCorrectCallback);

  ros::Subscriber position_incorrect = nh.subscribe("/position_incorrect", 10, positionInCorrectCallback);

  ros::Subscriber bottom_suction_success = nh.subscribe("/bottom_suction_success", 10, bottomSuctionSuccessCallback);

  ros::Subscriber bottom_suction_failed = nh.subscribe("/bottom_suction_failed", 10, bottomSuctionFailedCallback);

  ros::Subscriber start_removing_bags = nh.subscribe("/start_removing_bags", 10, startRemovingBagsCallback);

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
  float home[] = {-0.10, 0.45, 0.8, 0, 3.142, 7.10};

  // count for zones
  // we use these zones for picking algorithm
  // zone 1 -- topmost stack row , zone 4-- lowest stack row
  int zone = 1;

  // count for enclosure
  // we use the enclosure count to know which enclosure to place in
  // ***************************************************************************************************************
  int enclosure = 1;
// *****************************************************************************************************************
  while (ros::ok()){

  // loop for pick and place operations for 24 plastic bags
  for (int i = 1; i < 25; i++){

    // ********home ***********
    if (i == 1){
      // %%%%%go to home position for camera detection
      movement(move_group, home[0], home[1], home[2], home[3], home[4], home[5]);
      // jointMove(move_group);
      ros::Duration(1).sleep();

    }
    // service proxy request initialize
    yolov5_ros::petri srv;

    // publish line to pick from for camera detection
    std_msgs::String msg;
    stringstream ss;
    ss << "depth estimation zone"<<" "<<zone;
    msg.data = ss.str();
    line_pub.publish(msg);
    ros::Duration(3).sleep(); // sleep for center point to be detected properly

    // service request and call - camera detection
    int detect_service_var = 1;
    srv.request.detect = detect_service_var;
    vector<float> transformedPointsPanda;

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

      // publish ready for picking
      std_msgs::Int32 ready_picking_msg;
      ready_picking_msg.data = 1;
      ready_picking_pub.publish(ready_picking_msg);
      ros::Duration(1).sleep();

      // based on zones different waypoint
      if (zone == 4) {
        // %%%%%% go to detected object position - directly above
        movement(move_group, transformedPointsPanda[0], 0.4, 0.4, 0, 3.142, 6.90);//2.76, 0.35, 3.6);//0.4

        // %%%%%% go to pick descent operation 1
        movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.2, 0, 3.142, 6.90);

        // %%%%%% go to pick desent operation 2
        movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.12, 0, 3.142, 6.90);
        ros::Duration(3).sleep();//sleep for picking
      } else if (zone  == 3) {
        // %%%%%% go to detected object position - directly above
        movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.49, 2.76, 0.35, 3.6);//0.4

        // %%%%%% go to pick descent operation 1
        movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.25, 0, 3.142, 7.050);

        // %%%%%% go to pick descent operation 2
        movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.13, 0, 3.142, 6.95);

        ros::Duration(3).sleep();//sleep for picking

      } else if (zone == 2) {
        // %%%%%% go to detected object position - directly above
        movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.49, 2.76, 0.35, 3.6);//0.4

        // %%%%%%%%% go to pick descent operation 1
        movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.23, 0, 3.142, 6.50);

          // %%%%%%%%% go to pick descent operation 2
        movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.15, 0, 3.142, 6.50);
        ros::Duration(3).sleep();
      } else {
        // %%%%%% go to detected object position - directly above
        movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.49, 2.76, 0.35, 3.6);//0.4

        ros::Duration(1).sleep();
        // for zone 1
        // %%%%%%%%% go to pick descent operation 1
        movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.153, 0, 3.142, 6.90);//0.1

        ros::Duration(3).sleep();
      }
    } else {
      ROS_ERROR("failed to call service");
      return 1;
      // %%%%% go to place operation 2 - rise
    }

    // ********************pick rise operations *********
    // based on zones different waypoint
    if (zone  == 4){

      // %%%%%%%%% go topick rise operation 1
      movement(move_group, transformedPointsPanda[0], 0.5, 0.25, 0, 3.142, 7.050);

      // %%%%%%%%%% go to pick rise operation 2
      movement(move_group, transformedPointsPanda[0], 0.5, 0.50, 2.79, 0.35, 3.6);
    }
    else if (zone == 3){
      // %%%%%%%% go to pick rise operation 1
      movement(move_group, transformedPointsPanda[0], 0.5, 0.25, 0, 3.142, 7.050);

      // %%%%% go to pick rise operation 2
      movement(move_group, transformedPointsPanda[0], 0.5, 0.50, 2.79, 0.29, 3.7);

    } else {
      // for zone 1,2
      // %%%%%%%% go to pick rise operation 1
      // movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.25, 2.79, 0.29, 3.6);

      // %%%%% go to pick rise operation 1
      movement(move_group, transformedPointsPanda[0], transformedPointsPanda[1], 0.50, 2.79, 0.29, 3.6);

    }




    // *********** midpoint operation *******************

    if (enclosure == 1){
      // %%%%%% go to midpoint operation 1
      movement(move_group, 0.0, 0.2, 0.83, 0, 3.142, 6.00);

      // %%%%%% go to midpoint operation 2
      movement(move_group, 0.20, 0.365, 0.850, 2.79, 0.95, 2.4);
    } else if (enclosure == 2){
      // %%%%%% go to midpoint operation 1
      movement(move_group, 0.0, 0.2, 0.83, 0, 3.142, 6.00);

      // %%%%%% go to midpoint operation 2
      movement(move_group, 0.30, 0.24, 0.850, 2.79, 0.95, 2.4);
    }
    else if (enclosure == 3) {
      // %%%%%% go to midpoint operation 1
      movement(move_group, 0.0, 0.2, 0.83, 0, 3.142, 6.00);
    } else if (enclosure == 4){
      // %%%%%% go to midpoint operation 1
      movement(move_group, 0.0, 0.2, 0.83, 0, 3.142, 6.00);

      // %%%%%% go to midpoint operation 2
      movement(move_group, 0.40, 0.1, 0.87, 2.79, 0.50, 2.4);//0.85
    } else if (enclosure == 5){
      // %%%%%% go to midpoint operation 1
      movement(move_group, 0.2, 0.2, 0.83, 0, 3.142, 6.00);

      // %%%%%% go to midpoint operation 2
      movement(move_group, 0.4, 0.0, 0.85, 2.79, 0.50, 2.4);

    } else if (enclosure == 6){
      // %%%%%% go to midpoint operation 1
      movement(move_group, 0.2, 0.2, 0.83, 0, 3.142, 6.00);

      // %%%%%% go to midpoint operation 2
      movement(move_group, 0.44, -0.16, 0.85, 2.79, 0.75, 2.1);
    } else if (enclosure == 7){

      // %%%%%% go to midpoint operation 1
      movement(move_group, 0.2, 0.2, 0.83, 0, 3.142, 6.00);

      // %%%%%% go to midpoint operation 2
      movement(move_group, 0.44, -0.16, 0.85, 2.79, 0.75, 2.1);

      // %%%%%% go to midpoint operation 2
      movement(move_group, 0.44, -0.27, 0.85, 2.79, 0.75, 2.1);
    }
    else if (enclosure == 8) {
      // %%%%%% go to midpoint operation 1
      movement(move_group, 0.2, 0.2, 0.83, 0, 3.142, 6.00);

      // %%%%%% go to midpoint operation 2
      movement(move_group, 0.4, -0.16, 0.85, 2.79, 0.75, 2.1);

      // %%%%%% go to midpoint operation 2
      movement(move_group, 0.4, -0.27, 0.85, 2.79, 0.75, 2.1);


    } else {
      // do nothing
    }


    // ********** place operation ************************
    if (enclosure == 1){
      // %%%%% go to place operation 1 - directly above
      movement(move_group, 0.52, 0.35, 0.80, 2.79, 0.95, 2.4);//0.80

      // %%%%% go to place operation 1 - descent
      movement(move_group, 0.52, 0.35, 0.60, 2.79, 0.50, 2.4);

      // %%%%% go to place operation 2 - descent
      movement(move_group, 0.52, 0.35, 0.39, 0, 3.142, 5.60);

      // publish over drop postion
      std_msgs::Int32 over_drop_position;
      over_drop_position.data = 1;
      over_drop_position_pub.publish(over_drop_position);
      ros::Duration(4).sleep();// sleep for suction release

      // %%%%% go to place operation 1 - rise
      movement(move_group, 0.52, 0.35, 0.60, 2.79, 0.75, 2.1);
    } else if (enclosure == 2){
      // %%%%% go to place operation 1 - directly above
      movement(move_group, 0.52,0.24, 0.81, 2.79, 0.95, 2.4);

      // %%%%% go to place operation 1 - descent
      movement(move_group, 0.52,0.24, 0.60, 2.79, 0.50, 2.4);

      // %%%%% go to place operation 2 - descent
      movement(move_group, 0.52,0.24, 0.39, 0, 3.142, 5.60);

      // publish over drop postion
      std_msgs::Int32 over_drop_position;
      over_drop_position.data = 1;
      over_drop_position_pub.publish(over_drop_position);
      ros::Duration(4).sleep();// sleep for suction release

      // %%%%% go to place operation 1 - rise
      movement(move_group, 0.52,0.24, 0.60, 2.79, 0.75, 2.1);
    }
    else if (enclosure == 3){
      // %%%%% go to place operation 1 - directly above
      movement(move_group, 0.50, 0.13, 0.80, 2.79, 0.95, 2.4);

      // %%%%% go to place operation 1 - descent
      movement(move_group, 0.50, 0.13, 0.60, 2.79, 0.50, 2.4);

      // %%%%% go to place operation 2 - descent
      movement(move_group, 0.50, 0.13, 0.39, 0, 3.142, 5.60);

      // publish over drop postion
      std_msgs::Int32 over_drop_position;
      over_drop_position.data = 1;
      over_drop_position_pub.publish(over_drop_position);
      ros::Duration(4).sleep();// sleep for suction release

      // %%%%% go to place operation 1 - rise
      movement(move_group, 0.50, 0.13, 0.60, 2.79, 0.75, 2.1);
    } else if (enclosure == 4){
      // %%%%% go to place operation 1 - directly above
      movement(move_group, 0.50, 0.024, 0.80, 2.79, 0.95, 2.4);

      // %%%%% go to place operation 1 - descent
      movement(move_group, 0.50, 0.024, 0.60, 2.79, 0.50, 2.4);

      // %%%%% go to place operation 2 - descent
      movement(move_group, 0.50, 0.024, 0.39, 0, 3.142, 5.60);

      // publish over drop postion
      std_msgs::Int32 over_drop_position;
      over_drop_position.data = 1;
      over_drop_position_pub.publish(over_drop_position);
      ros::Duration(4).sleep();// sleep for suction release

      // %%%%% go to place operation 1 - rise
      movement(move_group, 0.50, 0.024, 0.60, 2.79, 0.75, 2.1);
    } else if (enclosure == 5){
      // %%%%% go to place operation 1 - directly above
      movement(move_group, 0.49, -0.08, 0.80, 2.79, 0.95, 2.4);

      // %%%%% go to place operation 1 - descent
      movement(move_group, 0.49, -0.08, 0.60, 2.79, 0.50, 2.4);

      // %%%%% go to place operation 2 - descent
      movement(move_group, 0.49, -0.08, 0.39, 0, 3.142, 5.60);

      // publish over drop postion
      std_msgs::Int32 over_drop_position;
      over_drop_position.data = 1;
      over_drop_position_pub.publish(over_drop_position);
      ros::Duration(4).sleep();// sleep for suction release

      // %%%%% go to place operation 1 - rise
      movement(move_group, 0.49, -0.08, 0.60, 2.79, 0.75, 2.1);


    } else if (enclosure == 6){
      // %%%%% go to place operation 1 - directly above
      movement(move_group, 0.49, -0.189, 0.80, 2.79, 0.95, 2.4);

      // %%%%% go to place operation 1 - descent
      movement(move_group, 0.49, -0.189, 0.60, 2.79, 0.50, 2.4);

      // %%%%% go to place operation 2 - descent
      movement(move_group, 0.49, -0.189, 0.39, 0, 3.142, 5.60);

      // publish over drop postion
      std_msgs::Int32 over_drop_position;
      over_drop_position.data = 1;
      over_drop_position_pub.publish(over_drop_position);
      ros::Duration(4).sleep();// sleep for suction release

      // %%%%% go to place operation 1 - rise
      movement(move_group, 0.49, -0.189, 0.60, 2.79, 0.75, 2.1);


    } else if (enclosure == 7){
      // %%%%% go to place operation 1 - directly above
      movement(move_group, 0.46, -0.28, 0.7, 2.79, 0.95, 2.2);

      // %%%%% go to place operation 1 - descent
      movement(move_group, 0.49, -0.28, 0.60, 0, 3.142, 5.00);//2.79, 0.65, 4.0);

      // %%%%% go to place operation 2 - descent
      movement(move_group, 0.49, -0.28, 0.39, 0, 3.142, 5.00);

      // publish over drop postion
      std_msgs::Int32 over_drop_position;
      over_drop_position.data = 1;
      over_drop_position_pub.publish(over_drop_position);
      ros::Duration(4).sleep();// sleep for suction release

      // %%%%% go to place operation 1 - rise
      movement(move_group, 0.49, -0.28, 0.60, 0, 3.142, 5.00);

      // %%%%%% go to place operation 2 - rise
      movement(move_group, 0.2, 0.0, 0.85, 2.79, 0.75, 2.1);
    }
    else if (enclosure == 8 ){
      // %%%%% go to place operation 1 - directly above
      movement(move_group, 0.46, -0.39, 0.75, 2.79, 0.95, 2.0);

      // %%%%% go to place operation 1 - descent
      movement(move_group, 0.49, -0.39, 0.60, 0, 3.142, 5.00);

      // publish over drop postion
      std_msgs::Int32 over_drop_position;
      over_drop_position.data = 1;
      over_drop_position_pub.publish(over_drop_position);
      ros::Duration(4).sleep();// sleep for suction release

      // %%%%% go to place operation 1 - rise
      movement(move_group, 0.49, -0.39, 0.60, 0, 3.142, 5.00);

    } else {
      // do nothing
    }

    // **************** change petri pick zone *****************
    if (i%6 == 0){
      zone = zone + 1;
      // reset zone
      if (zone == 5) {
        zone = 1;
      }
    }




    // *************** picking cut bag operation *****************
    if (i % 8 == 0){//i >= 4 ){
      // %%%%%% go to midpoint operation 1
      movement(move_group, 0.20, -0.310, 0.83, 0, 3.142, 5.00);


      // // %%%%%% go to midpoint operation 2
      // movement(move_group, 0.000, -0.410, 0.7, 0, 3.142, 5.00);



      // publish finished dropping cycle
      std_msgs::Int32 finished_dropping_cycle_msg;
      finished_dropping_cycle_msg.data = 1;
      finished_dropping_cycle_pub.publish(finished_dropping_cycle_msg);
      ros::Duration(1).sleep();

      // while (!startRemovingBags){
      //   cout<<"system not ready for bag removal";
      // }

      if (true){
        {
          while (true){
            char char1;
            cout<<"\n<";
            cout<<"Press a key";
            cin.get(char1);
            char char2 = 'a';
            char char3 = 'b';
            char char4 = 'c';
            char char5 = 'd';
            if (char1 == char2){
              // %%%%% pick cut operation enclosure 8

              // publish ready for picking for suction on
              std_msgs::Int32 ready_picking_msg;
              ready_picking_msg.data = 1;
              ready_picking_pub.publish(ready_picking_msg);
              ros::Duration(4).sleep();

              // %%%%% go to pick cut  operation 1 - directly above
              movement(move_group, 0.485, -0.40, 0.60, 0, 3.142, 5.00);

              // %%%%% go to pick cut  operation 2 - descent
              movement(move_group, 0.485, -0.40, 0.32, 0, 3.142, 5.00);
            }
           else if (char1 == char3){
             // // %%%%% go to pick cut  operation 1 - rise
             movement(move_group, 0.485, -0.40, 0.60, 0, 3.142, 5.00);
           }
           else if (char1 == char4){
             // // %%%%% go to pick cut  operation 1 - midpoint
             movement(move_group, 0.20, -0.210, 0.83, 0, 3.142, 5.00);
             //
             // // %%%%% go to pick cut  operation 1 - drop
             movement(move_group, 0.00, -0.350, 0.83, 2.79, 0.95, 1.0);

             // publish over drop postion to off suction for drop
             std_msgs::Int32 over_drop_position;
             over_drop_position.data = 1;
             over_drop_position_pub.publish(over_drop_position);
             ros::Duration(7).sleep();// sleep for suction release


           }
           else if (char1 == char5){
             break;
           }
           else {
           }
          }
        }

        {


          while (true){
            char char1;
            cout<<"\n<";
            cout<<"Press a key";
            cin.get(char1);
            char char2 = 'a';
            char char3 = 'b';
            char char4 = 'c';
            char char5 = 'd';
            if (char1 == char2){
              // %%%%% pick cut operation enclosure 7

              // publish ready for picking for suction on
              std_msgs::Int32 ready_picking_msg;
              ready_picking_msg.data = 1;
              ready_picking_pub.publish(ready_picking_msg);
              ros::Duration(4).sleep();

              // %%%%% go to pick cut  operation 1 - directly above
              movement(move_group, 0.49, -0.295, 0.60, 0, 3.142, 5.00);

              // %%%%% go to pick cut  operation 2 - descent
              movement(move_group, 0.49, -0.295, 0.29, 0, 3.142, 5.00);
            }
           else if (char1 == char3){
             // // %%%%% go to pick cut  operation 1 - rise
             movement(move_group, 0.49, -0.295, 0.60, 0, 3.142, 5.00);
           }
           else if (char1 == char4){
             // // %%%%% go to pick cut  operation 1 - midpoint
             movement(move_group, 0.20, -0.210, 0.83, 0, 3.142, 5.00);
             //
             // // %%%%% go to pick cut  operation 1 - drop
             movement(move_group, 0.00, -0.350, 0.83, 2.79, 0.95, 1.0);

             // publish over drop postion to off suction for drop
             std_msgs::Int32 over_drop_position;
             over_drop_position.data = 1;
             over_drop_position_pub.publish(over_drop_position);
             ros::Duration(7).sleep();// sleep for suction release


           }
           else if (char1 == char5){
             break;
           }
           else {
           }
          }
        }
      {
        while (true){
          char char1;
          cout<<"\n<";
          cout<<"Press a key";
          cin.get(char1);
          char char2 = 'a';
          char char3 = 'b';
          char char4 = 'c';
          char char5 = 'd';
          if (char1 == char2){
            // %%%%% pick cut operation enclosure 6

            // publish ready for picking for suction on
            std_msgs::Int32 ready_picking_msg;
            ready_picking_msg.data = 1;
            ready_picking_pub.publish(ready_picking_msg);
            ros::Duration(4).sleep();

            // %%%%% go to pick cut  operation 1 - directly above
            movement(move_group, 0.49, -0.182, 0.60, 0, 3.142, 5.60);

            // %%%%% go to pick cut  operation 2 - descent
            movement(move_group, 0.49, -0.182, 0.29, 0, 3.142, 5.20);//0.29
          }
         else if (char1 == char3){
           // // %%%%% go to pick cut  operation 1 - rise
           movement(move_group, 0.49, -0.182, 0.60, 0, 3.142, 5.60);
         }
         else if (char1 == char4){
           // // %%%%% go to pick cut  operation 1 - midpoint
           movement(move_group, 0.20, -0.210, 0.83, 0, 3.142, 5.00);
           //
           // // %%%%% go to pick cut  operation 1 - drop
           movement(move_group, 0.00, -0.350, 0.83, 2.79, 0.95, 1.0);

           // publish over drop postion to off suction for drop
           std_msgs::Int32 over_drop_position;
           over_drop_position.data = 1;
           over_drop_position_pub.publish(over_drop_position);
           ros::Duration(7).sleep();// sleep for suction release


         }
         else if (char1 == char5){
           break;
         }
         else {
         }
        }
      }

      {
        while (true){
          char char1;
          cout<<"\n<";
          cout<<"Press a key";
          cin.get(char1);
          char char2 = 'a';
          char char3 = 'b';
          char char4 = 'c';
          char char5 = 'd';
          if (char1 == char2){
            // %%%%% pick cut operation enclosure 5

            // publish ready for picking for suction on
            std_msgs::Int32 ready_picking_msg;
            ready_picking_msg.data = 1;
            ready_picking_pub.publish(ready_picking_msg);
            ros::Duration(4).sleep();

            // %%%%% go to pick cut  operation 1 - directly above
            movement(move_group, 0.49, -0.079, 0.60, 0, 3.142, 5.60);

            // %%%%% go to pick cut  operation 2 - descent
            movement(move_group, 0.49, -0.079, 0.29, 0, 3.142, 5.20);
          }
         else if (char1 == char3){
           // // %%%%% go to pick cut  operation 1 - rise
           movement(move_group, 0.49, -0.079, 0.60, 0, 3.142, 5.60);
         }
         else if (char1 == char4){
           // // %%%%% go to pick cut  operation 1 - midpoint
           movement(move_group, 0.20, -0.210, 0.83, 0, 3.142, 5.00);
           //
           // // %%%%% go to pick cut  operation 1 - drop
           movement(move_group, 0.00, -0.350, 0.83, 2.79, 0.95, 1.0);

           // publish over drop postion to off suction for drop
           std_msgs::Int32 over_drop_position;
           over_drop_position.data = 1;
           over_drop_position_pub.publish(over_drop_position);
           ros::Duration(7).sleep();// sleep for suction release

         }
         else if (char1 == char5){
           break;
         }
         else {
         }
        }
      }
      {
        while (true){
          char char1;
          cout<<"\n<";
          cout<<"Press a key";
          cin.get(char1);
          char char2 = 'a';
          char char3 = 'b';
          char char4 = 'c';
          char char5 = 'd';
          if (char1 == char2){
            // %%%%% pick cut operation enclosure 4
            // %%%%% go to pick cut  operation 1 - directly above
            movement(move_group, 0.50, 0.025, 0.60, 0, 3.142, 5.60);

            // %%%%% go to pick cut  operation 2 - descent
            movement(move_group, 0.50, 0.025, 0.29, 0, 3.142, 5.60);//0.29
          }
         else if (char1 == char3){
           // // %%%%% go to pick cut  operation 1 - rise
           movement(move_group, 0.50, 0.025, 0.60, 0, 3.142, 5.60);
         }
         else if (char1 == char4){
           // // %%%%% go to pick cut  operation 1 - midpoint
           movement(move_group, 0.20, -0.210, 0.83, 0, 3.142, 5.00);
           //
           // // %%%%% go to pick cut  operation 1 - drop
           movement(move_group, 0.00, -0.350, 0.83, 2.79, 0.95, 1.0);

         }
         else if (char1 == char5){
           break;
         }
         else {
         }
        }
      }
      {
        while (true){
          char char1;
          cout<<"\n<";
          cout<<"Press a key";
          cin.get(char1);
          char char2 = 'a';
          char char3 = 'b';
          char char4 = 'c';
          char char5 = 'd';
          if (char1 == char2){
            // %%%%% pick cut operation enclosure 3
            // %%%%% go to pick cut  operation 1 - directly above
            movement(move_group, 0.50, 0.14, 0.60, 0, 3.142, 5.60);

            // %%%%% go to pick cut  operation 2 - descent
            movement(move_group, 0.50, 0.14, 0.29, 0, 3.142, 5.60);
          }
         else if (char1 == char3){
           // // %%%%% go to pick cut  operation 1 - rise
           movement(move_group, 0.50, 0.14, 0.60, 0, 3.142, 5.60);
         }
         else if (char1 == char4){
           // // %%%%% go to pick cut  operation 1 - midpoint
           movement(move_group, 0.20, -0.210, 0.83, 0, 3.142, 5.00);
           //
           // // %%%%% go to pick cut  operation 1 - drop
           movement(move_group, 0.00, -0.350, 0.83, 2.79, 0.95, 1.0);

         }
         else if (char1 == char5){
           break;
         }
         else {
         }

        }
      }
      {
        while (true){
          char char1;
          cout<<"\n<";
          cout<<"Press a key";
          cin.get(char1);
          char char2 = 'a';
          char char3 = 'b';
          char char4 = 'c';
          char char5 = 'd';
          if (char1 == char2){
            // %%%%% pick cut operation enclosure 2
            // %%%%% go to pick cut  operation 1 - directly above
            movement(move_group, 0.50, 0.24, 0.60, 0, 3.142, 5.60);

            // %%%%% go to pick cut  operation 2 - descent
            movement(move_group, 0.50, 0.24, 0.29, 0, 3.142, 5.60);
          }
         else if (char1 == char3){
           // // %%%%% go to pick cut  operation 1 - rise
           movement(move_group, 0.50, 0.24, 0.60, 0, 3.142, 5.60);
         }
         else if (char1 == char4){
           // // %%%%% go to pick cut  operation 1 - midpoint
           movement(move_group, 0.20, -0.210, 0.83, 0, 3.142, 5.00);
           //
           // // %%%%% go to pick cut  operation 1 - drop
           movement(move_group, 0.00, -0.350, 0.83, 2.79, 0.95, 1.0);

         }
         else if (char1 == char5){
           break;
         }
         else {
         }
        }
      }
      {
        // midpoint
        // movement(move_group,  0.20, 0.350, 0.83, 2.79, 0.95, 2.0);
        while (true){

          char char1;
          cout<<"\n<";
          cout<<"Press a key";
          cin.get(char1);
          char char2 = 'a';
          char char3 = 'b';
          char char4 = 'c';
          char char5 = 'd';
          if (char1 == char2){
            // %%%%% pick cut operation enclosure 1

            // publish ready for picking for suction on
            std_msgs::Int32 ready_picking_msg;
            ready_picking_msg.data = 1;
            ready_picking_pub.publish(ready_picking_msg);
            ros::Duration(4).sleep();

            // %%%%% go to pick cut  operation 1 - directly above
            movement(move_group, 0.526, 0.34, 0.60, 0, 3.142, 5.60);

            // %%%%% go to pick cut  operation 2 - descent
            movement(move_group,  0.526, 0.34, 0.29, 0, 3.142, 5.60);//0.29
          }
         else if (char1 == char3){
           // // %%%%% go to pick cut  operation 1 - rise
           movement(move_group,  0.526, 0.34, 0.60, 0, 3.142, 5.60);
         }
         else if (char1 == char4){
           // // %%%%% go to pick cut  operation 1 - midpoint
           movement(move_group, 0.20, -0.210, 0.83, 0, 3.142, 5.00);
           //
           // // %%%%% go to pick cut  operation 1 - drop
           movement(move_group, 0.00, -0.350, 0.83, 2.79, 0.95, 1.0);

           // publish over drop postion to off suction for drop
           std_msgs::Int32 over_drop_position;
           over_drop_position.data = 1;
           over_drop_position_pub.publish(over_drop_position);
           ros::Duration(7).sleep();// sleep for suction release


         }
         else if (char1 == char5){
           break;
         }
         else {
         }
        }
      }
    } else {
      {
        while (true){
          char char1;
          cout<<"\n<";
          cout<<"Press a key";
          cin.get(char1);
          char char2 = 'a';
          char char3 = 'b';
          char char4 = 'c';
          char char5 = 'd';
          if (char1 == char2){
            // %%%%% pick cut operation enclosure 1

            // publish ready for picking for suction on
            std_msgs::Int32 ready_picking_msg;
            ready_picking_msg.data = 1;
            ready_picking_pub.publish(ready_picking_msg);
            ros::Duration(4).sleep();

            // %%%%% go to pick cut  operation 1 - directly above
            movement(move_group, 0.526, 0.34, 0.60, 0, 3.142, 5.60);

            // %%%%% go to pick cut  operation 2 - descent
            movement(move_group,  0.526, 0.34, 0.29, 0, 3.142, 5.60);//0.29
          }
         else if (char1 == char3){
           // // %%%%% go to pick cut  operation 1 - rise
           movement(move_group,  0.526, 0.34, 0.60, 0, 3.142, 5.60);
         }
         else if (char1 == char4){
           // // %%%%% go to pick cut  operation 1 - midpoint
           movement(move_group, 0.20, -0.210, 0.83, 0, 3.142, 5.00);
           //
           // // %%%%% go to pick cut  operation 1 - drop
           movement(move_group, 0.00, -0.350, 0.83, 2.79, 0.95, 1.0);

           // publish over drop postion to off suction for drop
           std_msgs::Int32 over_drop_position;
           over_drop_position.data = 1;
           over_drop_position_pub.publish(over_drop_position);
           ros::Duration(7).sleep();// sleep for suction release


         }
         else if (char1 == char5){
           break;
         }
         else {
         }
        }
      }
    }

      //  publish message for remove cut bag
      std_msgs::Int32 removing_bag_status_check_msg;
      removing_bag_status_check_msg.data = 1;
      removing_bag_status_check_pub.publish(removing_bag_status_check_msg);
      ros::Duration(1).sleep();
      // resetDone = false;
    }

    // increment enclosure count
    enclosure = enclosure + 1;

    // reset enclosure count
    if (enclosure > 8){
      enclosure = 1;
    }


    // ************* home position *************
    // %%%%%go to home position for camera detection
    movement(move_group, home[0], home[1], home[2], home[3], home[4], home[5]);
    // jointMove(move_group);

    ros::Duration(1).sleep();

    // reset system ready value
    // systemReady = false;

  }
  ros::spinOnce();
  }
  return 0;
}


// // rotate midpoint operation
// const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("panda_arm");
// moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
// // Next get the current set of joint values for the group.
// std::vector<double> joint_group_positions;
// current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
// // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
// joint_group_positions[0] = -tau / 4;  // -1/6 turn in radians
// move_group.setJointValueTarget(joint_group_positions);
// moveit::planning_interface::MoveGroupInterface::Plan my_plan;
// bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
// move_group.execute(my_plan);
// ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
// move_group.execute(my_plan);
// if (!success){
//   ROS_INFO("Shutting down due to planner failure");
//   ros::shutdown();
// }


// const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("panda_arm");
// moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
// std::vector<double> joint_group_positions;
// current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
// for (const auto &value : joint_group_positions) {
//       cout << value << " ";
//   }
// cout << endl;


// void jointMove (moveit::planning_interface::MoveGroupInterface& move_group) {

//   // rotate midpoint operation
//   const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("panda_arm");
//   moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
//   // Next get the current set of joint values for the group.
//   std::vector<double> joint_group_positions; //= {1.04939, -0.0299414, 0.713093, -1.09439, 0.0230849, 1.07092, -2.18504 };
//   current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//   // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
//   joint_group_positions[0] = 1.04939;  // -1/6 turn in radians
//   joint_group_positions[1] = -0.0299414;  // -1/6 turn in radians
//   joint_group_positions[2] = 0.713093;  // -1/6 turn in radians
//   joint_group_positions[3] = -1.09439;  // -1/6 turn in radians
//   joint_group_positions[4] = 0.0230849;  // -1/6 turn in radians
//   joint_group_positions[5] = 1.07092;  // -1/6 turn in radians
//   joint_group_positions[6] = -2.18504;  // -1/6 turn in radians
//   move_group.setJointValueTarget(joint_group_positions);
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   move_group.execute(my_plan);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan (joint goal) %s", success ? "" : "FAILED");
//   move_group.execute(my_plan);
//   if (!success){
//     ROS_INFO("Shutting down due to planner failure");
//     ros::shutdown();
//   }
// }
