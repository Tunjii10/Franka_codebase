#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
z_pub = rospy.Publisher('/depth_D435', Point, queue_size=1)

class depth:
    def __init__(self):
        rospy.init_node('depth_node')
        self.x = 0
        self.y = 0
        self.z = 0
        self.col=0
        self.row=0
        self.xy_subscriber = rospy.Subscriber('/center_point', Point, self.xy_callback)
        self.depth_subscriber = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.z_depth = rospy.Publisher('depth_D435', Point, queue_size=10)

    def xy_callback(self,data):
        self.x = int(data.x)
        self.y = int(data.y)

    def depth_callback(self,image):
        global new_x
        bridge = CvBridge()
        cv2_frame = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        depth_frame = cv2_frame.astype(np.float32)

        # Specify the pixel for the depth
        self.row = rospy.get_param('~row', self.y)
        self.col = rospy.get_param('~col', self.x)

        # Get the depth value at the specified pixel
        depth_value = depth_frame[self.row, self.col]
        depth_value = depth_value*0.001
        z_depth_msg= Point()
        # z_depth_msg.z= depth_value
        z_depth_msg.z= depth_value
        self.z_depth.publish(z_depth_msg)

if __name__ == '__main__':
    node = depth()
    rospy.spin()
