#!/usr/bin/env python3
import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32
from detection_msgs.msg import BoundingBox, BoundingBoxes
from yolov5_ros.srv import petri, petriResponse


# add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox

global bd_box_x, bd_box_y

@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        self.z_subscriber = rospy.Subscriber('/depth_D435', Point, self.z_callback)
        self.z_subscriber = rospy.Subscriber('/classid', Int32, self.id_callback)
        self.xy_subscriber = rospy.Subscriber('/center_point', Point, self.xy_callback)
        self.z_depth = 0
        self.y_data = 0
        self.x_data = 0
        self.x=0
        self.y=0
        self.z=0
        self.id=0
        self.center_x = 0
        self.center_y = 0
        self.points = []
        self.m_x=0
        self.m_y=0
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        # Initialize weights
        weights = rospy.get_param("~weights")
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device","")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup()  # warmup

        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.callback, queue_size=1
            )

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic"), BoundingBoxes, queue_size=10
        )
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=10
            )

        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def id_callback(self,id_data):
        self.id = id_data.data
        # print(self.id)

    def z_callback(self,z_data):
        self.z_depth = z_data.z #depth from the camera

    def xy_callback(self,xy_data): #Center point of QR
        self.x_data = xy_data.x
        self.y_data = xy_data.y

    def callback(self, data):
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        im, im0 = self.preprocess(im)
        # Run inference
        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.id, self.agnostic_nms, max_det=30
        )
        # Process predictions
        det = pred[0].cpu().numpy()
        bounding_boxes = BoundingBoxes()
        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det):
                self.center_x = (int(xyxy[0])+int(xyxy[2]))*0.5 #calculating the center pont of bounding box
                self.center_y = (int(xyxy[1])+int(xyxy[3]))*0.5

                # if (self.center_y-40) <= self.y_data < (self.center_y+40):
                self.points.append((self.center_x, self.center_y))
                sorted_xy = sorted(self.points, key=lambda point: point[0])
                self.m_x,self.m_y = sorted_xy[-1] #sorting bounding box in order to track it
                color=(0,255,255)
                thickness=2

                # lines to visualize the detection area
                # cv2.line(im0,(50,int(self.y_data)+40),(600,int(self.y_data)+40),color,thickness)
                # cv2.line(im0,(50,int(self.y_data)-40),(600,int(self.y_data)-40),color,thickness)

                bounding_box = BoundingBox()
                c = int(cls)
                bounding_box.Class = self.names[c]
                bounding_box.probability = conf

                self.z= 0.56# add depth data manualy if needed or comment it if taking from camera
                # self.z = 0.8
                # convering to cartesian coordinates
                # self.x = self.z * 1.003759398
                # self.y = self.z * 0.752819549

                self.x = self.z * 1.003759398
                self.y = self.z * 0.752819549
                bounding_box.x_center = float((((xyxy[0]+xyxy[2])/2)-320)*(self.x/640))
                bounding_box.y_center = float((((xyxy[1]+xyxy[3])/2)-240)*(self.y/480))

                bounding_boxes.bounding_boxes.append(bounding_box)

                # Annotate the image
                if self.publish_image or self.view_image:  # Add bbox to image
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))
                # else:
                #     pass

            # Stream results
            im0 = annotator.result()
            # sending tracked estimated picking location
            cv2.circle(im0, tuple((int(self.m_x),int(self.m_y))), 10, (0, 255, 0), -1)
            self.m_x = float((self.m_x-320)*(self.x/640))
            self.m_y = float((self.m_y-240)*(self.y/480))

        # Publish prediction
        self.pred_pub.publish(bounding_boxes)

        global bd_box_y, bd_box_x
        bd_box_x = self.m_x
        bd_box_y = self.m_y

        # Publish & visualize images
        if self.view_image:
            cv2.imshow(str(0), im0)
            cv2.waitKey(1)  # 1 millisecond
            self.points.clear()
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0

    def detect(self, req):
        print(req)
        global bd_box_y, bd_box_x
        while bd_box_y == 0  and bd_box_x == 0:
            print("not detecting")
        res = [bd_box_x, bd_box_y]
        return petriResponse(res)

if __name__ == "__main__":

    check_requirements(exclude=("tensorboard", "thop"))
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    rospy.Service('petri_service', petri, detector.detect)
    print("Ready to detect.")
    rospy.spin()
