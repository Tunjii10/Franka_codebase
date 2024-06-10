#!/usr/bin/env python

import rospy
import tf.transformations
import numpy as np
import franka_gripper.msg
import sys
import actionlib
import time

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from tf.transformations import quaternion_slerp
from automation.srv import *


initial_pose = PoseStamped()
initial_pose_found = False
pose_to_pub = PoseStamped()
start_pose = PoseStamped()
 



def franka_state_callback(msg):
    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
   
    
    initial_pose.pose.orientation.x = initial_quaternion[0]
    initial_pose.pose.orientation.y = initial_quaternion[1]
    initial_pose.pose.orientation.z = initial_quaternion[2]
    initial_pose.pose.orientation.w = initial_quaternion[3]
    initial_pose.pose.position.x = msg.O_T_EE[12]
    initial_pose.pose.position.y = msg.O_T_EE[13]
    initial_pose.pose.position.z = msg.O_T_EE[14]
    global initial_pose_found
    initial_pose_found = True
    # print("**Initial Pose**")
    # print(initial_pose)



def run(state_sub, pose_pub, rate, client, req):

    while not initial_pose_found:
        rospy.sleep(1)
    client.wait_for_server()
    success = False
    start_pose = initial_pose
    start_time = rospy.get_rostime()

    num_steps = int(5000 * 10)
    step_size = 1.0 / num_steps
    for i in range(num_steps + 1):
        t = i * step_size
        pose_to_pub.pose.position.x =  (1 - t) * start_pose.pose.position.x + t * req.posX
        pose_to_pub.pose.position.y = (1 - t) * start_pose.pose.position.y + t * req.posY
        pose_to_pub.pose.position.z =   (1 - t) * start_pose.pose.position.z + t * req.posZ
        start_quat = (start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w)
        goal_quat = (req.ortX, req.ortY, req.ortZ, req.ortW)
        interp_quat = quaternion_slerp(start_quat, goal_quat, t)
        pose_to_pub.pose.orientation.x = interp_quat[0]
        pose_to_pub.pose.orientation.y = interp_quat[1]
        pose_to_pub.pose.orientation.z = interp_quat[2]
        pose_to_pub.pose.orientation.w = interp_quat[3]
        pose_to_pub.header.frame_id = "panda_link0"
        pose_to_pub.header.stamp = rospy.Time.now()
        pose_pub.publish(pose_to_pub)
 

    rospy.sleep(5)
    goal = franka_gripper.msg.GraspGoal()
    goal.width = 0.02#max width with move action 0.08
    goal.epsilon.inner = 0.005
    goal.epsilon.outer = 0.005
    goal.speed = 0.05
    goal.force = 3
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()

    # Prints out the result of executing the action
    # if result.success:
    #     print("grasped")
    # else:
    #     return success 
   

                
    start_pose = initial_pose


    for i in range(num_steps + 1):
        t = i * step_size
        pose_to_pub.pose.position.x =  (1 - t) * start_pose.pose.position.x + t * 0.500000000000
        pose_to_pub.pose.position.y = (1 - t) * start_pose.pose.position.y + t * 0.600000000000
        pose_to_pub.pose.position.z =   (1 - t) * start_pose.pose.position.z + t * 0.300000000000
        start_quat = (start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w)
        goal_quat = (req.ortX, req.ortY, req.ortZ, req.ortW)
        interp_quat = quaternion_slerp(start_quat, goal_quat, t)
        pose_to_pub.pose.orientation.x = interp_quat[0]
        pose_to_pub.pose.orientation.y = interp_quat[1]
        pose_to_pub.pose.orientation.z = interp_quat[2]
        pose_to_pub.pose.orientation.w = interp_quat[3]
        pose_to_pub.header.frame_id = "panda_link0"
        pose_to_pub.header.stamp = rospy.Time.now()
        pose_pub.publish(pose_to_pub)
 

    rospy.sleep(5)
    goal = franka_gripper.msg.GraspGoal()
    goal.width = 0.08#max width with move action 0.08
    goal.epsilon.inner = 0.005
    goal.epsilon.outer = 0.005
    goal.speed = 0.05
    goal.force = 1
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    # if result.success:
    #     print("grasped")
    # else:
    #     return success 
    

    success = True
    return success   


def handle_impedance_pose(req):
    success = False
    print("Requested Position \n")
    print(req)
    state_sub = rospy.Subscriber("franka_state_controller/franka_states",
                                FrankaState, franka_state_callback)
    pose_pub = rospy.Publisher(
        "cartesian_impedance_example_controller/equilibrium_pose",  PoseStamped, queue_size=100)
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

    rate = rospy.Rate(2000.0)
    rospy.sleep(2)
    connections = pose_pub.get_num_connections()
    rospy.loginfo('Connections: %d', connections)
          
    success = run(state_sub, pose_pub, rate, client, req)
    return success
        
        
    # rospy.spin()
def impedance_server():
    rospy.init_node("impedance_pose_server", anonymous=True)
    s = rospy.Service('impedance_pose', movePose, handle_impedance_pose)
    print("Ready for pose commands")
    rospy.spin()

          

if __name__ == "__main__":
    try: 
        impedance_server()
        
    except rospy.ROSInterruptException:
		pass
    


