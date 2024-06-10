# Overview
The nr_robot folder contains code for running the Franka Panda Robot. In this folder, franka_ros and a panda_moveit_config codes are also added. The Automation folder contains custom codes for working with the franka robot.

**This readMe explains the custom code folder.**

# Launch Files
**1.**  Robo.launch - Used to launch the robot with moveit configs. It also launches the publishers for the calibrated (hand-eye) transforms.

**2.**  Gazebo.launch - Launches the robot in gazebo simulation with moveit configs 

# Msg Files
This contains custom yolo_detection messages

# Scripts
**1.** demo.cpp, pick_cut.cpp, pilz.cpp  - Contains code for panda pick and place with LIN motion planners and camera object detection

**2.** detection.cpp - Contains code for panda pick and place with collision objects and camera object detection

**3.** detection.py - Contains python code for panda pick and place 

**4.** impedance_python.py - Contains code for impedance control implemented as a service using python

**5.** move_pose_server.cpp - Contains code for panda pick and place with collision objects and gripper commands

**6.** srv_client.py - Contains code for server client for impedance control

**7.** teleop.cpp - Panda robot Rviz marker control 