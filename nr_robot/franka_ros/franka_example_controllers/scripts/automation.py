#!/usr/bin/env python3

# IMPORT MODULES

# import ros modules
from rospy_tutorials.msg import Floats
from std_msgs.msg import Time, Header, UInt32, UInt8, Bool, String, Float64
from sensor_msgs.msg import Image,PointCloud2,PointField,CameraInfo 
from matplotlib import pyplot as plt
from cv_bridge import CvBridge
from franka_msgs.msg import FrankaState
#from ros_numpy import point_cloud2
from geometry_msgs.msg import PoseStamped,TwistStamped
import rospy
import tf
import tf.transformations as tr
from tf.transformations import quaternion_matrix
# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import math  

import time
import threading
import sys

# tf.transformations.quaternion_matrix()
# GLOBAL VARIABLES DECLARATION

status = '0'
input_key = '0'

phantom_detection_flag = False
vessel_detection_flag = False
pos_1 = False
pos_2 = False
flag = True

target_pose_detection_1 = PoseStamped() # phantom pose in camera space
target_pose_detection_2 = PoseStamped() # vessel pose in US space
pose_storage_left_arm = PoseStamped() # container to store useful poses
pose_storage_right_arm = PoseStamped() # container to store useful poses
left_arm_desired_end_effector_pose = PoseStamped() # desired end effector pose (N.B. link 8 not the probe) for the probe arm wrt base link frame (link 0) 
T_EE_left = Float64() # the same but in homogeneous coordinates
right_arm_desired_end_effector_pose = PoseStamped() # desired end effector pose (N.B. link 8 not the needle) for the needle arm wrt base link frame (link 0) 
T_EE_right = Float64() #  the same but in homogeneous coordinates
#impedance_value = 0.0

home_left_arm_end_effector_pose = PoseStamped()
home_right_arm_end_effector_pose = PoseStamped()


# FUNCTIONS

# da vettore ed angolo a matrice di rotazione

def axis_angle_to_rot_mat(r,tetha):
	# r = [rx,ry,rz]
	# tetha in radians
	tetha =  0.52 # radian
	rx = r[0]
	ry = r[1]
	rz = r[2]
	ct = math.cos(tetha)
	st = math.sin(tetha)
	T_support = [
			[ rx*rx*(1-ct) + ct        , rx*ry*(1-ct) - rz*st     , rx*rz*(1-ct) + ry*st     , 0],
			[ rx*ry*(1-ct) + rz*st     , ry*ry*(1-ct) + ct        , ry*rz*(1-ct) - rx*st     , 0],
			[ rx*rz*(1-ct) - ry*st     , ry*rz*(1-ct) + rx*st     , rz*rz*(1-ct) + ct        , 0],
			[ 0                        , 0                        , 0                        , 1]
			]	

# da vettore a matrice

def vect_to_mat(vect):
	T = [	[vect[0],  vect[4],  vect[8],    vect[12]],
			[vect[1],  vect[5],  vect[9],    vect[13]],
			[vect[2],  vect[6],  vect[10],   vect[14]],
			[vect[3],  vect[7],  vect[11],   vect[15]]
		]
	return T

# da pos a matrix

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

# non potevo passare alla funzione il quaternione facendo pos_to_mat(target_pose_detection_1.pose.orientation,target_pose_detection_1.pose.direction) 
# perchè non riusciva a leggere i valori non essendo orientation e direction dei classici array, ma dei dati strutturati con campi .x,.y etc s... 

def pos_to_mat_1(pos):
	qx = pos.pose.orientation.x
	qy = pos.pose.orientation.y
	qz = pos.pose.orientation.z
	qw = pos.pose.orientation.w
	tx = pos.pose.position.x
	ty = pos.pose.position.y
	tz = pos.pose.position.z
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

# inversa di matrice 

def inv(T):
	R = [
		[T[0][0],T[0][1],T[0][2]],
		[T[1][0],T[1][1],T[1][2]],
		[T[2][0],T[2][1],T[2][2]]
	]
	trasl =[T[0][3],T[1][3],T[2][3]]
	R_min_inv = [
		[-T[0][0],-T[1][0],-T[2][0]],
		[-T[0][1],-T[1][1],-T[2][1]],
		[-T[0][2],-T[1][2],-T[2][2]]
	]
	trasl_new = np.dot(R_min_inv,trasl)
	T_new = [
		[T[0][0],T[1][0],T[2][0],trasl_new[0]],
		[T[0][1],T[1][1],T[2][1],trasl_new[1]],
		[T[0][2],T[1][2],T[2][2],trasl_new[2]],
		[      0,      0,      0,           1]
	]
	return T_new

# functions to read the desired command by a separate thread

def handling_input(inp):
    print('Got {}'.format(inp))


def background():
	global input_key, status
	print('You are in {} mode. Till status 3 included everything should work properly.'.format(status))
	print('0 - waiting mode')
	print('1 - READ THE CURRENT ROBOTS STATES AND SAVE AS HOME POSES')
	print('2 - DESIRED END EFFECTOR POSE COMPUTATION')
	print('3 - Send US to the detected ROI')
	print('4 - US SCANNING MODELS')
	print('5 - Send the needle to the probe tip pose')
	print('6 - EMPTY')
	print('7 - parameter restoration')
	while True:
		inp = input()
		handling_input(inp)
		input_key = inp
		status = input_key
		if input_key=='0':
			print('You are in {} mode (waiting mode).'.format(status))
		elif input_key=='1':
			print('You are in {} mode (READ THE CURRENT ROBOTS STATES AND SAVE AS HOME POSES).'.format(status))
		elif input_key=='2':
			print('You are in {} mode (DESIRED END EFFECTOR POSE COMPUTATION, this pose (if published) makes the probe tip reach the target).'.format(status))
		elif input_key=='3':
			print('You are in {} mode (Send US to the detected ROI).'.format(status))
		elif input_key=='4':
			print('You are in {} mode (US SCANNING MODELS).'.format(status))
		elif input_key=='5':
			print('You are in {} mode (Send the needle to the probe tip pose).'.format(status))
		elif input_key=='6':
			print('You are in {} mode (EMPTY).'.format(status))
		elif input_key=='7':
			print('You are in {} mode (parameter restoration).'.format(status))
		else:
			print('Undefined mode')


# CALLBACKS

def callback_target_pose_detection_1(data):		# phantom pose
	global target_pose_detection_1
	target_pose_detection_1.pose.position.x = data.pose.position.x
	target_pose_detection_1.pose.position.y = data.pose.position.y
	target_pose_detection_1.pose.position.z = data.pose.position.z
	target_pose_detection_1.pose.orientation.x = data.pose.orientation.x
	target_pose_detection_1.pose.orientation.y = data.pose.orientation.y
	target_pose_detection_1.pose.orientation.z = data.pose.orientation.z
	target_pose_detection_1.pose.orientation.w = data.pose.orientation.w
	target_pose_detection_1.header.stamp = data.header.stamp 
	target_pose_detection_1.header.frame_id = data.header.frame_id

def callback_left_arm_franka_state(data):
	global T_EE_left 
	T_EE_left = data.O_T_EE

def callback_right_arm_franka_state(data):
	global T_EE_right 
	T_EE_right = data.O_T_EE
	
#def callback_cathbotpro_impedance(data):
#	global impedance_value 
#	impedance_value = data.data[0]


# CORE FUNCTION 

def core_node():
	global status, input_key, phantom_detection_flag, vessel_detection_flag, pos_1, pos_2, flag, target_pose_detection_1, target_pose_detection_2, pose_storage_left_arm, pose_storage_right_arm,left_arm_desired_end_effector_pose, T_EE_left, right_arm_desired_end_effector_pose, T_EE_right 

	# SETUP STEPS

	# node initialisation

	rospy.init_node('status_node', anonymous=True)
	rate = rospy.Rate(1000)

	# publishers

	pub_1 = rospy.Publisher('/left_arm/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=1)	# /left_arm/desiredCartesianPose	#  /pippo_left
	pub_2 = rospy.Publisher('/right_arm/desiredCartesianPose', PoseStamped, queue_size=1)	#  /pippo_righct
	pub_3 = rospy.Publisher('/test_topic_for_left_arm', PoseStamped, queue_size=1)	#  /pippo_right
	pub_4 = rospy.Publisher('/test_topic_for_right_arm', PoseStamped, queue_size=1)	#  /pippo_right
	# subscribers

	rospy.Subscriber("/test", PoseStamped, callback_target_pose_detection_1)	# /target_pose_detection
	rospy.Subscriber("/left_arm/franka_state_controller/franka_states", FrankaState, callback_left_arm_franka_state)
	rospy.Subscriber("/right_arm/franka_state_controller/franka_states", FrankaState, callback_right_arm_franka_state)
#	rospy.Subscriber("/cathbotpro_impedance", Float64MultiArray, callback_cathbotpro_impedance)

	# transform listeners

	listener_1 = tf.TransformListener()
	listener_2 = tf.TransformListener()
	listener_3 = tf.TransformListener()	
	listener_4 = tf.TransformListener()	
	listener_5 = tf.TransformListener()
	listener_6 = tf.TransformListener()
	listener_7 = tf.TransformListener()
	listener_8 = tf.TransformListener()
	listener_9 = tf.TransformListener()
	listener_10 = tf.TransformListener()
	listener_11 = tf.TransformListener()
	listener_12 = tf.TransformListener()

	rospy.sleep(2.) # we have to wait in order to let ros activate all the nodes and all the topics to be published otherwise we might face 

	# loop

	while not rospy.is_shutdown():

		# VARIABLES UPDATES

		# tf collecting

		try:
			(trans_1,rot_1) = listener_1.lookupTransform('left_arm_link0', 'camera_color_frame', rospy.Time(0)) # L 0-camera_color
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		try:
			(trans_2,rot_2) = listener_2.lookupTransform('left_arm_link0', 'left_arm_link8', rospy.Time(0)) # L 0-8
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		try:
			(trans_3,rot_3) = listener_3.lookupTransform('left_arm_link0', 'left_arm_probe_tip', rospy.Time(0)) # L 0-probe_tip
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		try:
			(trans_4,rot_4) = listener_4.lookupTransform('left_arm_link8', 'left_arm_probe_tip', rospy.Time(0)) # L 8-probe_tip
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		try:
			(trans_5,rot_5) = listener_5.lookupTransform('left_arm_probe_tip', 'left_arm_link8', rospy.Time(0)) # L probe_tip-8
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		try:
			(trans_6,rot_6) = listener_6.lookupTransform('left_arm_probe_tip', 'left_arm_link0', rospy.Time(0)) # L probe_tip-0
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		try:
			(trans_7,rot_7) = listener_7.lookupTransform('right_arm_link0', 'right_arm_link8', rospy.Time(0)) # R 0-8
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		try:
			(trans_8,rot_8) = listener_8.lookupTransform('right_arm_link0', 'right_arm_needle_tip', rospy.Time(0)) # R 0-needle_tip
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		try:
			(trans_9,rot_9) = listener_9.lookupTransform('right_arm_link8', 'right_arm_needle_tip', rospy.Time(0)) # R 8-needle_tip
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		try:
			(trans_10,rot_10) = listener_10.lookupTransform('right_arm_needle_tip', 'right_arm_link8', rospy.Time(0)) # R needle_tip-8
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		try:
			(trans_11,rot_11) = listener_11.lookupTransform('right_arm_needle_tip', 'right_arm_link0', rospy.Time(0)) # R needle_tip-0
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		try:
			(trans_12,rot_12) = listener_12.lookupTransform('right_arm_link0', 'camera_color_frame', rospy.Time(0)) # R 0-camera_color
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue


		# identity matrix definition

		I = [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]

		# data formatting

		T_1 = pos_to_mat(rot_1,trans_1)	   #L 0-camera_color
		T_2 = pos_to_mat(rot_2,trans_2)    #L 0-8
		T_3 = pos_to_mat(rot_3,trans_3)    #L 0-probe_tip
		T_4 = pos_to_mat(rot_4,trans_4)    #L 8-probe_tip
		T_5 = pos_to_mat(rot_5,trans_5)    #L probe_tip-8
		T_6 = pos_to_mat(rot_6,trans_6)    #L probe_tip-0
		T_7 = pos_to_mat(rot_7,trans_7)    #R 0-8
		T_8 = pos_to_mat(rot_8,trans_8)    #R 0-needle_tip
		T_9 = pos_to_mat(rot_9,trans_9)    #R 8-needle_tip
		T_10 = pos_to_mat(rot_10,trans_10) #R needle_tip-8
		T_11 = pos_to_mat(rot_11,trans_11) #R needle_tip-0
		T_12 = pos_to_mat(rot_12,trans_12) #R 0-camera_color

		T_EE_left_matrix = vect_to_mat(T_EE_left)
		T_EE_right_matrix = vect_to_mat(T_EE_right)


		# STATUS 

		# READ THE CURRENT ROBOTS STATES AND SAVE AS HOME POSES  ! 
		if status == '1':
			home_left_arm_end_effector_pose = mat_to_pos(T_EE_left_matrix,"left_arm_link0")
			home_right_arm_end_effector_pose = mat_to_pos(T_EE_right_matrix,"right_arm_link0")
			#pub_1.publish(home_left_arm_end_effector_pose)
			#rospy.loginfo(home_left_arm_end_effector_pose)
			#pub_2.publish(home_right_arm_end_effector_pose)
			#rospy.loginfo(home_right_arm_end_effector_pose)
			#t_t = T_7
			#print(t_t)
			rate.sleep()

		# # TBD
		# if status == '1':
		# 	# left_arm_desired_end_effector_pose = mat_to_pos(T_EE_left_matrix,"left_arm_link0")
		# 	# right_arm_desired_end_effector_pose = mat_to_pos(T_EE_right_matrix,"right_arm_link0")
		# 	print("ciao")
		# 	# pub_1.publish(left_arm_desired_end_effector_pose)
		# 	# # rospy.loginfo(left_arm_desired_end_effector_pose)
		# 	# pub_2.publish(right_arm_desired_end_effector_pose)
		# 	# # rospy.loginfo(right_arm_desired_end_effector_pose)
		# 	# rate.sleep()

		# DESIRED END EFFECTOR POSE COMPUTATION (it makes the probe tip reach the target)
		if status == '2':

			T_target_pose_detection_1 = pos_to_mat_1(target_pose_detection_1)
			T_result_temp = np.dot(T_1,T_target_pose_detection_1) # target pose of the probe tip in RoF link 0
			pos_temp = mat_to_pos(T_result_temp,"left_arm_link0")
			pos_temp.pose.orientation = home_left_arm_end_effector_pose.pose.orientation
			T_result_temp_temp = pos_to_mat_1(pos_temp)
			T_result = np.dot(T_result_temp_temp,inv(T_4)) # in currect frame roto-translation frome desired probe tip to desired link 8

			left_arm_desired_end_effector_pose.pose.position.x = T_result[0][3]
			left_arm_desired_end_effector_pose.pose.position.y = T_result[1][3]
			left_arm_desired_end_effector_pose.pose.position.z = T_result[2][3]
			# here we know that the tf between the probe tip and the link 8 has null rotation so we can directly exploit the orientation of the link 8 stored in the home pose mode 
			# q mighty be used if we wanted to alling the probe tip frame to the target
			left_arm_desired_end_effector_pose.pose.orientation = home_left_arm_end_effector_pose.pose.orientation
			left_arm_desired_end_effector_pose.header.stamp = rospy.Time.now()
			left_arm_desired_end_effector_pose.header.frame_id = "left_arm_link0" 

			pose_storage_left_arm = left_arm_desired_end_effector_pose
			T_pose_storage_left_arm = pos_to_mat_1(pose_storage_left_arm)
			#print("WE ARE IN STATUS 2")
			pub_3.publish(left_arm_desired_end_effector_pose)	# in this state left_arm_desired_end_effector_pose == pose_storage_left_arm
			#rospy.loginfo(left_arm_desired_end_effector_pose)			
			# print(T_pose_storage_left_arm)
			rate.sleep()
		 
		# Send US to the detected ROI  
		if status == '3':    
			# print('inv(T_4)')
			# print(inv(T_4))
			left_arm_desired_end_effector_pose = pose_storage_left_arm	# pose_storage_left_arm == left_arm_desired_end_effector_pose stored in state 2
			left_arm_desired_end_effector_pose.header.stamp = rospy.Time.now()
			left_arm_desired_end_effector_pose.header.frame_id = "left_arm_link0" 
			pub_1.publish(left_arm_desired_end_effector_pose)
			# rospy.loginfo(left_arm_desired_end_effector_pose)

			rate.sleep()


		#  US SCANNING MODELS 
		if status == '4':

	    	# in order to scan for a vessel we will perform just a rotation exploiting as center of rotation the probe tip
	    	# rotation of +-30° around the x axis of the probe tip reference frame

			#T_pos1 = [
			#		[    1,      0,      0, 0],
			#		[    0,      0,     -1, 0],
			#		[    0,      1,      0, 0],
			#		[    0,      0,      0, 1]
			#		]	

			T_pos1 = [
					[    1,      0,      0, 0],
					[    0,  0.866,  0.500, 0],
					[    0, -0.500,  0.866, 0],
					[    0,      0,      0, 1]
					]

			# traslazione lungo l' asse x del link 8 nella home pose 

			#T_posx = [
			#		[    ,      0,      0, 0.1],
			#		[    ,      0,      0,   0],
			#		[    ,      0,      0,   0],
			#		[    ,      0,      0,   1]
			#		]

			#T_pos1 = [
			#		[    1,      0,      0, 0],
			#		[    0,  0.866, -0.500, 0],
			#		[    0,  0.500,  0.866, 0],
			#		[    0,      0,      0, 1]
			#		]

			T_pos2 = [
					[    1,      0,      0, 0],
					[    0,  0.866,  0.500, 0],
					[    0, -0.500,  0.866, 0],
					[    0,      0,      0, 1]
					]
			
			T_new_1_desired_tip_pose_temp = np.dot(T_pose_storage_left_arm,T_4) # mi riporto alla posizione desiderata della tip T_4
			T_new_1_desired_tip_pose_temp_temp = np.dot(T_new_1_desired_tip_pose_temp,T_pos1) # applico la rotazione T_pos1
			T_new_1_desired_tip_pose = np.dot(T_new_1_desired_tip_pose_temp_temp,inv(T_4)) # mi riporto sul link 8 inv(T_4)
			print('pose_storage_left_arm')
			print(pose_storage_left_arm)
			print('T_pose_storage_left_arm')
			print(T_pose_storage_left_arm)
			print('T_new_1_desired_tip_pose_temp')
			print(T_new_1_desired_tip_pose_temp)
			print('T_new_1_desired_tip_pose_temp_temp')
			print(T_new_1_desired_tip_pose_temp_temp)
			print('T_new_1_desired_tip_pose')
			print(T_new_1_desired_tip_pose)

			T_new_2_desired_tip_pose_temp = np.dot(T_pose_storage_left_arm,T_4) # mi riporto alla posizione desiderata della tip
			T_new_2_desired_tip_pose_temp_temp = np.dot(T_new_2_desired_tip_pose_temp,T_pos2) # applico la rotazione
			T_new_2_desired_tip_pose = np.dot(T_new_2_desired_tip_pose_temp_temp,inv(T_4)) # mi riporto sul link 8

			if (vessel_detection_flag == False and pos_1 == False) :

				new_1_desired_tip_pose = mat_to_pos(T_new_1_desired_tip_pose,"left_arm_link0")
				left_arm_desired_end_effector_pose = new_1_desired_tip_pose
				left_arm_desired_end_effector_pose.header.stamp = rospy.Time.now()
				#pos_1 = True

			if (vessel_detection_flag == False and pos_1 == True and pos_2 == False) :
				new_2_desired_tip_pose = mat_to_pos(T_new_2_desired_tip_pose,"left_arm_link0")
				left_arm_desired_end_effector_pose = new_2_desired_tip_pose
				left_arm_desired_end_effector_pose.header.stamp = rospy.Time.now()
				#pos_2 = True

			if (vessel_detection_flag == False and pos_1 == True and pos_2 == True) :

				left_arm_desired_end_effector_pose = T_pose_storage_left_arm
				left_arm_desired_end_effector_pose.header.stamp = rospy.Time.now()

			pub_1.publish(left_arm_desired_end_effector_pose)
			#rospy.loginfo(left_arm_desired_end_effector_pose)
			pub_2.publish(right_arm_desired_end_effector_pose)
			#rospy.loginfo(right_arm_desired_end_effector_pose)
			rate.sleep()

		if status == '5':

			T_right_result_temp = np.dot(T_12,T_target_pose_detection_1) # T_12
			pos_right_temp = mat_to_pos(T_right_result_temp,"right_arm_link0")

			T_support = pos_to_mat_1(home_right_arm_end_effector_pose)
			T_support_1 = np.dot(T_support,T_9)
			pos_support = mat_to_pos(T_support_1,"right_arm_link0")

			pos_right_temp.pose.orientation = pos_support.pose.orientation

			T_right_result_temp_temp = pos_to_mat_1(pos_right_temp)
			T_right_result = np.dot(T_right_result_temp_temp,inv(T_9)) # inv(T_9)
	

			right_arm_desired_end_effector_pose.pose.position.x = T_right_result[0][3]
			right_arm_desired_end_effector_pose.pose.position.y = T_right_result[1][3]
			right_arm_desired_end_effector_pose.pose.position.z = T_right_result[2][3]
			right_arm_desired_end_effector_pose.pose.orientation = home_right_arm_end_effector_pose.pose.orientation
			right_arm_desired_end_effector_pose.header.stamp = rospy.Time.now()
			right_arm_desired_end_effector_pose.header.frame_id = "right_arm_link0" 

			pose_storage_right_arm = right_arm_desired_end_effector_pose
			T_pose_storage_right_arm = pos_to_mat_1(pose_storage_right_arm)

			# pub_1.publish(left_arm_desired_end_effector_pose)
			# rospy.loginfo(left_arm_desired_end_effector_pose)
			pub_2.publish(right_arm_desired_end_effector_pose)
			#rospy.loginfo(right_arm_desired_end_effector_pose)

			right_arm_desired_end_effector_pose = mat_to_pos(T_right_result_temp_temp,"left_arm_link0")
			right_arm_desired_end_effector_pose.header.stamp = rospy.Time.now()			
			pub_4.publish(right_arm_desired_end_effector_pose)
			#rospy.loginfo(right_arm_desired_end_effector_pose)

			rate.sleep()



		if status == '6':
			print("Not implemented.")

		if status == '7':
			print("Parameters restoration.")
			phantom_detection_flag = False
			vessel_detection_flag = False
			pos_1 = False
			pos_2 = False

# MAIN

if __name__ == '__main__':

	t = threading.Thread(target=background)
	t.daemon = True
	t.start()

	try:
		core_node()
	except rospy.ROSInterruptException:
		pass
