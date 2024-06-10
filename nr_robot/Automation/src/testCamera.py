#!/usr/bin/env python3

import rospy
import tf
import numpy as np
import franka_gripper.msg
import sys
import actionlib
import time

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from tf.transformations import quaternion_slerp,euler_from_matrix, quaternion_from_matrix, euler_from_quaternion

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


initial_pose = PoseStamped()
initial_pose_found = False
pose_to_pub = PoseStamped()
pose_to_pub2 = PoseStamped()
point_to_pub = Point()
 


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
    angles = euler_from_quaternion([initial_pose.pose.orientation.x, initial_pose.pose.orientation.y,initial_pose.pose.orientation.z,initial_pose.pose.orientation.w])   
    print("****angles****")
    print(angles)
    print("****positions****")
    print(initial_pose.pose.position)
    point_to_pub.x = initial_pose.pose.position.x
    point_to_pub.y = initial_pose.pose.position.y
    point_to_pub.z = initial_pose.pose.position.z
    z_pub.publish(point_to_pub)

def movement(x_link0, y_link0, z_link0, speed, x_ang, y_ang, z_ang):
    start_pose = initial_pose
    start_time = rospy.get_rostime()

    num_steps = int(speed* 10)
    step_size = 1.0 / num_steps
    rotation_matrix = tf.transformations.euler_matrix( x_ang, y_ang, z_ang)
    goal_quat = quaternion_from_matrix(rotation_matrix)
    
    for i in range(num_steps + 1):
        t = i * step_size
        pose_to_pub.pose.position.x =  (1 - t) * start_pose.pose.position.x + t * x_link0
        pose_to_pub.pose.position.y = (1 - t) * start_pose.pose.position.y + t *  y_link0
        pose_to_pub.pose.position.z =   (1 - t) * start_pose.pose.position.z + t * z_link0
        start_quat = (start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w)
        # goal_quat = (0.706217934154, 0.692907087411, 0.0907858929149, -0.113551395157)
        # rotation_matrix = tf.transformations.euler_matrix(-3.14159265359, 0.0, 0.0)
        # goal_quat = quaternion_from_matrix(rotation_matrix)
        # pose_to_pub.header.frame_id = "panda_link0"
        # interp_quat = quaternion_slerp(start_quat, goal_quat, t)
        pose_to_pub.pose.orientation.x = goal_quat[0]
        pose_to_pub.pose.orientation.y = goal_quat[1]
        pose_to_pub.pose.orientation.z = goal_quat[2]
        pose_to_pub.pose.orientation.w = goal_quat[3]
        pose_to_pub.header.frame_id = "panda_link0"
        pose_to_pub.header.stamp = rospy.Time.now()

        pose_pub.publish(pose_to_pub)


if __name__ == "__main__":
    try: 
        rospy.init_node('transform_example')
        rate = rospy.Rate(1000.0)
        state_sub = rospy.Subscriber("franka_state_controller/franka_states",
                                    FrankaState, franka_state_callback)
        pose_pub = rospy.Publisher(
            "cartesian_impedance_example_controller/equilibrium_pose",  PoseStamped, queue_size=100)
        marker_publisher = rospy.Publisher('visualization_marker', Marker)

        z_pub  = rospy.Publisher("frank_z",  Point, queue_size=10)
        rospy.sleep(2)
        connections = pose_pub.get_num_connections()
        rospy.loginfo('Connections: %d', connections)
 
        rospy.sleep(1)



        # coordinates from camera Camera coordinates
        x_camera =0.00  
        y_camera = 0.00  
        z_camera = 0.03


        # listener = tf.TransformListener()
        # # publishing to equilibrium pose only working with panda_link0
        # # so we change end effector tansform to panda_link0
        # listener.waitForTransform('panda_link0', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(4.0))
        # trans, rot = listener.lookupTransform('panda_link0', 'camera_color_optical_frame', rospy.Time(0))
        

        # # Homogeneous transformation matrix from end effector to panda_link0
        # camera_to_ee_matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))

        # # Object position in the end effector frame [x_ee, y_ee, z_ee]
        # # object_position_ee = np.array([x_ee, y_ee, z_ee, 1])
        # object_position_camera = np.array([x_camera, y_camera, z_camera, 1])
        # # Convert object position to the panda_link0 frame
        # object_position_link8 = np.dot(camera_to_ee_matrix, object_position_camera)

        # # Extract the transformed position [x, y, z]
        # x_link0, y_link0, z_link0, _ = object_position_link8
        # print(x_link0/10, y_link0/10, z_link0)
        while not initial_pose_found:
            rospy.sleep(1)
        # # cam pose
        # movement(0.3204576191514194, 0.0, 0.6727513856336, 8000, 3.14159265359, 0.0, 4.83321946706)
        # rospy.sleep(2)

        # # z axis place
        # movement(-0.1, 0.4, 0.40, 8000, 3.14159265359, 0.0, 4.83321946706)
        # rospy.sleep(2)

        # # direct top
        # movement(x_link0/100, y_link0/100, 0.30, 8000, 3.14159265359, 0.0, 4.83321946706)
        # rospy.sleep(2)
  
        # # pick
        # movement(-0.1, 0.6, 0.22, 5000, 3.14159265359, 0.0, 4.83321946706)
        # rospy.sleep(5)

        # # pick lift 1
        # movement(-0.1, 0.6, 0.5, 10000, 3.14159265359, 0.0, 4.83321946706)
        # rospy.sleep(2)

        # # pick lift 2
        # movement(-0.1, 0.6, 0.9, 10000, 3.14159265359, 0.0, 4.83321946706)
        # rospy.sleep(2)

        # # to place motion
        # movement(-0.8, 0.0, 0.9,20000, 3.14159265359, 0.0, 4.83321946706)
        # rospy.sleep(2)

        # # place descent 1
        # movement(-0.8, 0.0, 0.5, 20000, 3.14159265359, 0.0, 4.83321946706)
        # rospy.sleep(2)

        # # place descent 2
        # movement(-0.8, 0.0, 0.4, 20000, 3.14159265359, 0.0, 4.83321946706)
        # rospy.sleep(2)


        # return to base
        # movement(0.3204576191514194, 0.0, 0.6727513856336, 8000, 3.14159265359, 0.00, 3.14159265359)
        # rospy.sleep(2)
        rospy.spin()

        

        
    except rospy.ROSInterruptException:
        pass