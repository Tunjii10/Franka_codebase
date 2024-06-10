#!/usr/bin/env python3
import rospy
import cv2
from pyzbar.pyzbar import decode
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import String

rospy.init_node('qr_code_detection')

class QR_detection:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/camera/masked_image', Image, self.image_callback)
        self.center_pub = rospy.Publisher('center_point', Point, queue_size=1)
        self.image_sub = rospy.Subscriber('/string_zone', String, self.zone_selecter)
        self.target_content = ""
        self.bridge = CvBridge()

    def zone_selecter(self,data):
        self.target_content = data.data
        print(self.target_content)


    def image_callback(self,msg):
        cv2_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        frame = cv2_frame
        # Detect QR codes in the frame
        decoded_objects = decode(frame)

        for obj in decoded_objects:
            if obj.data.decode("utf-8") == self.target_content:
            # Draw a circle to mark the 4 corners of the detected QR code
                for point in obj.polygon:
                    cv2.circle(frame, tuple(point), 5, (0, 0, 255), -1)  # Red circle

                # Calculate the center point of the QR code
                center_point = np.mean(obj.polygon, axis=0, dtype=int)

                # Draw a circle to mark the center point
                cv2.circle(frame, tuple(center_point), 5, (0, 255, 0), -1)  # Green circle

                # Display the center point coordinates
                cv2.putText(frame, f"Center: {center_point[0]}, {center_point[1]}", (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                center_msg = Point()
                center_msg.x = center_point[0]
                center_msg.y = center_point[1]
                self.center_pub.publish(center_msg)

            # Display the frame
            cv2.imshow("QR Code Center Point", frame)
            cv2.waitKey(1)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    node = QR_detection()
    rospy.spin()
