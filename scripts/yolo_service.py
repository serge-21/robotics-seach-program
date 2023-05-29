#!/usr/bin/env python3
# coding=utf-8
import rospy
from yolov4 import Detector
from cv_bridge import CvBridge  
import cv2
from sensor_msgs.msg import Image
from second_coursework.msg import YOLODetection
from second_coursework.srv import YoloLastFrame, YoloLastFrameResponse

class YOLOv4ROSITR:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.frame = 0
        self.skip = 20  # We will process 1 of every self.skip frames
        
        # initialise the detector, subscriber, and the service 
        self.cam_subs = rospy.Subscriber("/camera/image", Image, self.img_callback)
        self.yolo_srv = rospy.Service('/detect_frame', YoloLastFrame, self.yolo_service)
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='/home/serge/ros_ws_2/src/second_coursework/config/coco.data')
    
    def img_callback(self, msg):
        if self.frame % self.skip == 0:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        else:
            self.cv_image = None
        self.frame = (self.frame + 1) % self.skip 
    
    # yolo service 
    def yolo_service(self, request):
        res = YoloLastFrameResponse()
        if self.cv_image is not None:
            cv_copy = self.cv_image.copy()
            cv_height, cv_width, _ = self.cv_image.shape
            img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
            for detection in detections:
                box = detection.left_x, detection.top_y, detection.width, detection.height
                # this is where we are printing it
                print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {box}')
                d = YOLODetection(detection.class_name, detection.class_confidence, detection.left_x, detection.top_y,
                                  detection.width, detection.height)
                # convert bbox to image space
                d.bbox_x = int((d.bbox_x/self.detector.network_width())*cv_width)
                d.bbox_y = int((d.bbox_y/self.detector.network_height())*cv_height)
                d.width = int((d.width/self.detector.network_width())*cv_width)
                d.height = int((d.height/self.detector.network_height())*cv_height)
                res.detections.append(d)
        return res

# if __name__ == '__main__':
#     rospy.init_node('yolo_ros_itr')
#     yolo_ros = YOLOv4ROSITR()
#     rospy.spin()