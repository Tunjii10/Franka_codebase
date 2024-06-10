The yolov5 and ROS were ingereated by a YOLOv5 package from https://github.com/mats-robotics/yolov5_ros.git.
READMEv5.md will provide more details about it.

The catkin_ws/src/yolov5_ros/src/detect.py # for detection and it modified to add tracking, depth, camera to cartesian coordinate conversion, communication between vision and robot. 

catkin_ws/src/yolov5_ros/src/preprocessing_bridge.py # for the conversion of rgb to gray image.

catkin_ws/src/yolov5_ros/src/qr_detection.py # for QR code detection and calculation of ceter point of QR detected.

catkin_ws/src/yolov5_ros/src/z_detection.py # for estimating the depth of detected QR code.

catkin_ws/src/yolov5_ros/src/yolov5/oct_9_best.pt # trained weight file for detection 

catkin_ws/src/detection_msgs/msg/BoundingBox.msg # detection custom message for picking 
# launch files
catkin_ws/src/yolov5_ros/launch/yolov5.launch  #for launching the YOLOv5.
# final lainch file
catkin_ws/src/yolov5_ros/launch/thermo_vision.launch # for launching the all the vision files. Before laucnhing need to install https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy