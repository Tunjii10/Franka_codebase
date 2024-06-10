#!/usr/bin/env python3

import rospy
import tf
import numpy as np
import franka_gripper.msg
import sys
import actionlib
import time
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion
from franka_msgs.msg import FrankaState
from tf.transformations import *
import tf.transformations as tr

from yolov5_ros.srv import petri, petriResponse

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


initial_pose = PoseStamped()
initial_pose_found = False
pose_to_pub = PoseStamped()
pose_to_pub2 = PoseStamped()

def show_text_in_rviz(pos_temp):
    marker_publisher.publish(pos_temp)

def pos_to_mat(quat,trasl):
	qx = quat[0]
	qy = quat[1]
	qz = quat[2]
	qw = quat[3]
	tx = trasl[0]
	ty = trasl[1]
	tz = trasl[2]
	q = [qx, qy, qz, qw]
	t = [tx, ty, tz]
	T_r = tr.quaternion_matrix(q)
	T_t  =tr.translation_matrix(t)
	T = T_r + T_t - [[1, 0, 0 , 0],[0, 1, 0 , 0],[0, 0, 1 , 0],[0, 0, 0 , 1]]
	return T


def pos_to_mat_1(x_camera, y_camera, z_camera):
	qx = 0#pos.pose.orientation.x
	qy = 0#pos.pose.orientation.y
	qz = 0#pos.pose.orientation.z
	qw = 1#pos.pose.orientation.w
	tx = x_camera#pos.pose.position.x
	ty = y_camera#pos.pose.position.y
	tz = z_camera#pos.pose.position.z
	q = [qx, qy, qz, qw]
	t = [tx, ty, tz]
	T_r = tr.quaternion_matrix(q)
	T_t  =tr.translation_matrix(t)
	T = T_r + T_t - [[1, 0, 0 , 0],[0, 1, 0 , 0],[0, 0, 1 , 0],[0, 0, 0 , 1]]
	return T


# da matrix a pos

def mat_to_pos(mat,frame_name):
	pos = PoseStamped()
	pos.pose.position.x = mat[0][3]
	pos.pose.position.y = mat[1][3]
	pos.pose.position.z = mat[2][3]
	q_temp = tr.quaternion_from_matrix(mat)
	q = tr.unit_vector(q_temp)
	pos.pose.orientation.x = q[0]
	pos.pose.orientation.y = q[1]
	pos.pose.orientation.z = q[2]
	pos.pose.orientation.w = q[3]
	pos.header.stamp = rospy.Time.now()
	pos.header.frame_id = frame_name
	return pos


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
    print(initial_pose)

def movement(speed, pos, orient):
    start_pose = initial_pose
    start_time = rospy.get_rostime()

    num_steps = int(speed* 10)
    step_size = 1.0 / num_steps

    if len(orient) == 3:
        rotation_matrix = tf.transformations.euler_matrix( orient[0], orient[1], orient[2])
        goal_quat = quaternion_from_matrix(rotation_matrix)
        orient  =  goal_quat
    # orient = Quaternion(orient[0], orient[1], orient[2], orient[3])
    start_orient = [start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w]
    print(orient)
    print(start_orient)
    orient_new = quaternion_multiply(orient, start_orient)

    old_x = 0
    old_y = 0
    old_z = 0

    for i in range(num_steps):
        t = float(i) / (num_steps-1)
        pose_to_pub.pose.position.x =  (1 - t) * start_pose.pose.position.x + t * pos[0]
        pose_to_pub.pose.position.y = (1 - t) * start_pose.pose.position.y + t *  pos[1]
        pose_to_pub.pose.position.z =   (1 - t) * start_pose.pose.position.z + t * pos[2]


        pose_to_pub.pose.orientation.x = orient_new[0]
        pose_to_pub.pose.orientation.y = orient_new[1]
        pose_to_pub.pose.orientation.z = orient_new[2]
        pose_to_pub.pose.orientation.w = orient_new[3]


        pose_to_pub.header.frame_id = "panda_link0"
        pose_to_pub.header.stamp = rospy.Time.now()
        pose_pub.publish(pose_to_pub)

        print("----pose published-----")
        print(pose_to_pub)
        print("----end pose published-----")
        if pose_to_pub.pose.position.x == old_x and pose_to_pub.pose.position.y == old_y and pose_to_pub.pose.position.z == old_z:
            print("stopped")
            break


        pose_to_pub.pose.position.x = old_x
        pose_to_pub.pose.position.y = old_y
        pose_to_pub.pose.position.z = old_z



    # rospy.sleep(2)
    print("----state callback-----")
    print(initial_pose)
    print("----end state callback-----")





    # print(pose_to_pub.pose)
    # show_text_in_rviz(pose_to_pub)

def detect(x_camera, y_camera, z_camera):
    listener = tf.TransformListener()
    # publishing to equilibrium pose only working with panda_link0
    # so we change end effector tansform to panda_link0
    listener.waitForTransform('panda_link0', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(4.0))
    trans, rot = listener.lookupTransform('panda_link0', 'camera_color_optical_frame', rospy.Time(0))


    T_target_pose_detection_1 = pos_to_mat_1(x_camera, y_camera, z_camera)
    T_1 = pos_to_mat(rot,trans)
    T_result_temp = np.dot(T_1,T_target_pose_detection_1)
    pos_temp = mat_to_pos(T_result_temp,"panda_link0")
    print (pos_temp.pose.position.x,pos_temp.pose.position.y, pos_temp.pose.position.z)
    return [pos_temp.pose.position.x,pos_temp.pose.position.y,pos_temp.pose.position.z]

def box(petri_Detect, pos_home, quat_home, pos_detect_home, quat_detect_home):
    x_camera = -0.06027261510178125
    y_camera = 0.025831120775062503
    z_camera = 0.0
    num = 1
    # zone
    for i in range(num):
        # pick
        resp = petri_Detect(1)
        # print service call response to screen
        print(resp.result)
        x_camera = resp.result[0]
        y_camera = resp.result[1]
        z_camera = resp.result[2]
        arr = detect(x_camera, y_camera, z_camera)
        pos_t = (arr[0], arr[1],  arr[2])#0.40)
        quat_t= (0.3622237207012903, 0.9320895572595912, 0.0016233448866874078,-0.0006309994548575625)
        movement(2000, pos_t, quat_t)
        pos_tilt = (arr[0], arr[1],  arr[2])#0.19)
        quat_tilt= (0.3698165170753044, 0.9288548265726251, 0.00821662343612614,-0.019923402077426496)
        # quat_tilt= (0.39472459160942525,  0.864290503089696, 0.11103097811698952, 0.2913186313061995)
        movement(5000, pos_tilt, quat_tilt)
        rospy.sleep(1)

        # pick rise 1st part
        pos_t = (arr[0], arr[1],  0.80)
        movement(4000, pos_t, quat_t)

        # # # # pick rise 2nd part
        # # # pos_t = (arr[0], arr[1],  0.950)
        # # # quat_t = (0.3969233021430672, 0.7554622198723355, 0.18641445824799882, 0.4868042484587736)
        # # # movement(3500, pos_t, quat_t)
        #
        # # place mid point
        # pos_place= (0.1717892411578945, 0.3253767856244797, 0.9515217903369477)
        # quat_place= (3.14159265359, 0, 2.00)
        # movement(3000, pos_place, quat_place)
        #
        # # place directly above
        # pos_place= (-0.2140738234, 0.4663263380584926, 0.920098406)
        # quat_place= (3.14159265359, 0, 3.00)#1.57)
        # movement(3000, pos_place, quat_place)
        #
        # # place directly settled
        #bd_box_z  pos_place= (-0.2140738234, 0.4759754556, 0.5)
        # quat_place= (3.14159265359, 0.0, 0.8)
        # movement(7000, pos_place, quat_place)
        # rospy.sleep(1)
        #
        # home
        # movement(4000, pos_home, quat_home)
        movement(4000, pos_detect_home, quat_detect_home)
        # if i != num-1:
        #     movement(3000, pos_detect_home, quat_detect_home)
        #     rospy.sleep(1)




if __name__ == "__main__":
    try:
        rospy.init_node('detection')
        rate = rospy.Rate(500.0)

        rospy.wait_for_service('/petri_service')

        state_sub = rospy.Subscriber("franka_state_controller/franka_states",
                                    FrankaState, franka_state_callback)
        pose_pub = rospy.Publisher(
            "cartesian_impedance_example_controller/equilibrium_pose",  PoseStamped, queue_size=100)
        marker_publisher = rospy.Publisher('marker', PoseStamped, queue_size=10)

        # initialise service proxy
        petri_Detect = rospy.ServiceProxy('/petri_service', petri)

        connections = pose_pub.get_num_connections()
        rospy.loginfo('Connections: %d', connections)

        rospy.sleep(2)

        # cordinates and orientations
        #home orientation:
        pos_home = (0.06885265922638234,0.007600406292727626,0.675742975736092)
        quat_home = (0.39051698780590594, 0.9202975564500481,0.01006373672315757,0.021156819946287313)

        #detect_home
        pos_detect_home = ( 0.2728995830087986, -0.03516536460051363, 0.8374402976709023)
        # quat_detect_home= (0.37938161845937964, 0.9144142500555361, -0.024811024453374644, -0.05103536022431737)
        quat_detect_home = (3.14159265359, 1.5, 4.2)

        # pos_detect_home = ( 0.12904655926727843,  -0.045988363016644655, 0.9104944218262105)
        # quat_detect_home= (0.36963400892159226,  0.9229728799315546, 0.04164122417637632, 0.09878142947182361)
        # pos_detect_home = ( 0.3824513267425953,-0.1578228798153445,0.7507923156710219)
        # quat_detect_home= (0.37685686373845484, 0.9261915695034868, -0.007020554390991775,0.009939449176225468)

        while not initial_pose_found:
            rospy.sleep(1)
        # show_text_in_rviz(pos_temp)
        # rospy.sleep(2)


        # home
        movement(2000, pos_home, quat_home)

        # detect home
        movement(2000, pos_detect_home, quat_detect_home)
        rospy.sleep(1)

        # box process
        # box(petri_Detect, pos_home, quat_home, pos_detect_home, quat_detect_home)

        rospy.spin()

    except rospy.ROSInterruptException:
	    pass
