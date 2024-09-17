#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node): #that will handle the image feed and perform object detection using YOLOv8.

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('~/helios/src/helios_recognition/scripts/yolov8n.pt') #The YOLO model is loaded from a specified file path (yolov8n.pt). 
                                                                                #This is the core detection model that performs inference.

        self.yolov8_inference = Yolov8Inference() #This is an instance of the Yolov8Inference message type, 
                                                    #which will store the YOLO detection results and be published to a ROS topic.

        self.subscription = self.create_subscription( #The node subscribes to the topic 'rgb_cam/image_raw' to get image data. 
            Image,                                      #When an image is received, it calls the camera_callback function.
            'rgb_cam/image_raw',
            self.camera_callback,
            10)
        self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1) #self.yolov8_pub publishes the detection results (bounding boxes, class names, etc.) 
                                                                                        #as a Yolov8Inference message on the /Yolov8_Inference topic.
        self.img_pub = self.create_publisher(Image, "/inference_result", 1) #self.img_pub publishes the annotated image (with bounding boxes drawn) as an Image message on /inference_result.

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8") #converts the ROS Image message into an OpenCV image
        results = self.model(img) #YOLO model processes the image (self.model(img)), returning detection results.

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg() #Sets a timestamp and frame ID for the Yolov8Inference message.

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")

        annotated_frame = results[0].plot() #The YOLO library annotates the image with bounding boxes.
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  #converted back to a ROS Image message.

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
