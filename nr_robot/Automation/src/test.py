#!/usr/bin/env python

import rospy
import tf
import numpy as np
import franka_gripper.msg
import sys
import actionlib
import time

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from tf.transformations import quaternion_slerp

initial_pose = PoseStamped()
initial_pose_found = False
pose_to_pub = PoseStamped()
pose_to_pub2 = PoseStamped()
 


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


def load_transformation_matrix():
    # loads the homogeneous transformation matrix from panda link8 (i.e the end effector) to the camera 
    franka_emc_path = "/home/favour/catkin_ws/src/Automation/src/franka_eMc.txt"  
    transformation_matrix = np.loadtxt(franka_emc_path)
    return transformation_matrix


if __name__ == "__main__":
    try: 
        rospy.init_node('test')
        rate = rospy.Rate(1000.0)
        state_sub = rospy.Subscriber("franka_state_controller/franka_states",
                                    FrankaState, franka_state_callback)
        pose_pub = rospy.Publisher(
            "cartesian_impedance_example_controller/equilibrium_pose",  PoseStamped, queue_size=100)

        rospy.sleep(2)
        connections = pose_pub.get_num_connections()
        rospy.loginfo('Connections: %d', connections)
 
        rospy.sleep(1)



        # coordinates from camera Camera coordinates
        x_camera = 0.1  
        y_camera = 0.0  
        # z_camera = 0.3

        # Create homogeneous vector
        homogeneous_vector = np.array([x_camera, y_camera, 0, 1])


        transformation_matrix = load_transformation_matrix()

        pose_end_effector = np.dot(np.linalg.inv(transformation_matrix), homogeneous_vector)
        print(pose_end_effector)
        x_ee, y_ee, z_ee, _ = pose_end_effector

        listener = tf.TransformListener()
        # publishing to equilibrium pose only working with panda_link0
        # so we change end effector tansform to panda_link0
        listener.waitForTransform('panda_link0', 'panda_link8', rospy.Time(0), rospy.Duration(4.0))
        trans, rot = listener.lookupTransform('panda_link0', 'panda_link8', rospy.Time(0))
        

        # Homogeneous transformation matrix from end effector to panda_link0
        ee_to_link0_matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))

        # Object position in the end effector frame [x_ee, y_ee, z_ee]
        # object_position_ee = np.array([x_ee, y_ee, z_ee, 1])
        object_position_ee = np.array([x_ee, y_ee, z_ee, 1])
        # Convert object position to the panda_link0 frame
        object_position_link0 = np.dot(ee_to_link0_matrix, object_position_ee)

        # Extract the transformed position [x, y, z]
        x_link0, y_link0, z_link0, _ = object_position_link0
        print(x_link0, y_link0, z_link0)

        while not initial_pose_found:
            rospy.sleep(1)

        # we use start pose as intial pose from state call back
        start_pose = initial_pose
        start_time = rospy.get_rostime()

        num_steps = int(5000 * 10)
        step_size = 1.0 / num_steps
        for i in range(num_steps + 1):
            # interpolate to goal position to control speed and smoothness
            t = i * step_size
            pose_to_pub.pose.position.x =  (1 - t) * start_pose.pose.position.x + t * x_link0
            pose_to_pub.pose.position.y = (1 - t) * start_pose.pose.position.y + t *  y_link0
            pose_to_pub.pose.position.z =   (1 - t) * start_pose.pose.position.z + t * 0.3
            start_quat = (start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w)
            # goal orientation
            goal_quat = (0.704974401273, 0.707871144545, 0.0339318815306, -0.0278955854127) 
            # goal_quat = (start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w)
            pose_to_pub.header.frame_id = "panda_link0"
            interp_quat = quaternion_slerp(start_quat, goal_quat, t)
            pose_to_pub.pose.orientation.x = interp_quat[0]
            pose_to_pub.pose.orientation.y = interp_quat[1]
            pose_to_pub.pose.orientation.z = interp_quat[2]
            pose_to_pub.pose.orientation.w = interp_quat[3]
            pose_to_pub.header.frame_id = "panda_link0"
            pose_to_pub.header.stamp = rospy.Time.now()
            pose_pub.publish(pose_to_pub)
    except rospy.ROSInterruptException:
		pass