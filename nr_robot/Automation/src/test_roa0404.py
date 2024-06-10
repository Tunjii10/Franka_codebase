import rospy
from std_msgs.msg import Int32



def image_callback(msg):
    # Convert ROS image message to OpenCV format
    print(msg.data)
if __name__ == '__main__':
    rospy.init_node('image_processing_node')
    rospy.Subscriber('/classid', Int32, image_callback)
    # masked_image_pub = rospy.Publisher('/camera/masked_image', Image, queue_size=1)

    rospy.spin()