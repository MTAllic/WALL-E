#!/usr/bin/env python3
#test file to test the topics

import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge() #Enables conversion between ROS and OpenCV image types.

class Camera_subscriber(Node): #A ROS2 node that subscribes to the camera feed ('rgb_cam/image_raw').

    def __init__(self):
        super().__init__('camera_subscriber')

        self.subscription = self.create_subscription(
            Image,
            'rgb_cam/image_raw',
            self.camera_callback,
            10)
        self.subscription 

    def camera_callback(self, data): #received image data is passed to the camera_callback function.
        global img
        img = bridge.imgmsg_to_cv2(data, "bgr8")

class Yolo_subscriber(Node): #A ROS2 node that subscribes to the YOLOv8 inference results on the /Yolov8_Inference topic.

    def __init__(self):
        super().__init__('yolo_subscriber')

        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10)
        self.subscription 

        self.cnt = 0

        self.img_pub = self.create_publisher(Image, "/inference_result_cv2", 1)

    def yolo_callback(self, data):
        #This processes the YOLOv8 inference results. For each detected object, it:
        #Logs the object’s class and bounding box coordinates.
        #Draws a rectangle around the object on the OpenCV image (img).
        global img
        for r in data.yolov8_inference:
        
            class_name = r.class_name
            top = r.top
            left = r.left
            bottom = r.bottom
            right = r.right
            yolo_subscriber.get_logger().info(f"{self.cnt} {class_name} : {top}, {left}, {bottom}, {right}")
            cv2.rectangle(img, (top, left), (bottom, right), (255, 255, 0))
            self.cnt += 1

        self.cnt = 0
        img_msg = bridge.cv2_to_imgmsg(img)  
        self.img_pub.publish(img_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    yolo_subscriber = Yolo_subscriber()
    camera_subscriber = Camera_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(yolo_subscriber)
    executor.add_node(camera_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = yolo_subscriber.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
