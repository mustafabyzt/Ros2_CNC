#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.listener_callback,
            10)
        
        self.bridge = CvBridge()
        
 
    def listener_callback(self, msg):
        self.get_logger().info('Receiving video frame')
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detected_frame = self.detect_objects(frame)
        cv2.imshow('Detected Objects', detected_frame)
        cv2.waitKey(1)
        
        

    def detect_objects(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)    
            
        return frame

def main(args=None):
    rclpy.init(args=args)
    object_detection = ObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
