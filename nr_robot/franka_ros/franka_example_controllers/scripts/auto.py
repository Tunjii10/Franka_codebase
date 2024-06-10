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

initial_pose = PoseStamped()
initial_pose_found = False
pose_to_pub = PoseStamped()
pose_to_pub2 = PoseStamped()
 

pick = True
place = False
home = False


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
    # print(initial_pose)


def run():

    rospy.init_node("auto", anonymous=True)
    rate = rospy.Rate(1000.0)
    state_sub = rospy.Subscriber("franka_state_controller/franka_states",
                                FrankaState, franka_state_callback)
    pose_pub = rospy.Publisher(
        "cartesian_impedance_example_controller/equilibrium_pose",  PoseStamped, queue_size=100)
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
    client.wait_for_server()
    rospy.sleep(2)
    connections = pose_pub.get_num_connections()
    rospy.loginfo('Connections: %d', connections)
          
    # listener = tf.TransformListener()
    # link_name = rospy.get_param("~link_name")

    # Get initial pose for the interactive marker

    
    while not initial_pose_found:
        rospy.sleep(1)
    # state_sub.unregister()
    
    global pick
    global place
    global home
    i=0
    while not rospy.is_shutdown():
        
    # while initial_pose_found:
        if pick :
            pose_to_pub.pose.position.x =  0.5 #initial_pose.pose.position.x 
            pose_to_pub.pose.position.y =  0.5 #initial_pose.pose.position.y 
            pose_to_pub.pose.position.z =   0.4
            pose_to_pub.pose.orientation.x = initial_pose.pose.orientation.x
            pose_to_pub.pose.orientation.y = initial_pose.pose.orientation.y
            pose_to_pub.pose.orientation.z = initial_pose.pose.orientation.z
            pose_to_pub.pose.orientation.w = initial_pose.pose.orientation.w
            pose_to_pub.header.frame_id = "panda_link0"
            pose_to_pub.header.stamp = rospy.Time.now()
            
            pose_pub.publish(pose_to_pub)
         
            # rate.sleep()
            print("picked--")
            # wait_for_pick()
            
            # pick = False
            # place = True
            # rate.sleep()
            
            # for t in range(20000):
                
            #     print(t)
            
            # time.sleep(3)

        if place:
            if i == 10000:
                break
            pose_to_pub.pose.position.x =  0.3 #initial_pose.pose.position.x 
            pose_to_pub.pose.position.y =  0.5 #initial_pose.pose.position.y 
            pose_to_pub.pose.position.z =   0.6
            pose_to_pub.pose.orientation.x = initial_pose.pose.orientation.x
            pose_to_pub.pose.orientation.y = initial_pose.pose.orientation.y
            pose_to_pub.pose.orientation.z = initial_pose.pose.orientation.z
            pose_to_pub.pose.orientation.w = initial_pose.pose.orientation.w
            pose_to_pub.header.frame_id = "panda_link0"
            pose_to_pub.header.stamp = rospy.Time.now()
            
            pose_pub.publish(pose_to_pub)
            

            rate.sleep()
            print("placed---")
            
            rate.sleep()
            
            

        if home :
            pose_to_pub.pose.position.x =  0 #initial_pose.pose.position.x 
            pose_to_pub.pose.position.y =  0 #initial_pose.pose.position.y 
            pose_to_pub.pose.position.z =   0.4
            pose_to_pub.pose.orientation.x = initial_pose.pose.orientation.x
            pose_to_pub.pose.orientation.y = initial_pose.pose.orientation.y
            pose_to_pub.pose.orientation.z = initial_pose.pose.orientation.z
            pose_to_pub.pose.orientation.w = initial_pose.pose.orientation.w
            pose_to_pub.header.frame_id = "panda_link0"
            pose_to_pub.header.stamp = rospy.Time.now()
            
            pose_pub.publish(pose_to_pub)
            

            rate.sleep()
            print("home---")
            home = False
            c = 10000
            for c in range(20000):
                
                print(c)

        # else:
        #     print("Doing Nothing")          

        pick = False
        place = True
        i+=1
        print(i)

        
        
    rospy.spin()


if __name__ == "__main__":
    try: 
        run()
        
    except rospy.ROSInterruptException:
		pass
    


