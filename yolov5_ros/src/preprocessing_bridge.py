#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def image_callback(msg):
    # Convert ROS image message to OpenCV format
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Convert masked image back to ROS image message
    masked_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

    # Publish the masked image
    masked_image_pub.publish(masked_msg)

if __name__ == '__main__':
    rospy.init_node('image_processing_node')
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    masked_image_pub = rospy.Publisher('/camera/masked_image', Image, queue_size=1)

    rospy.spin()
