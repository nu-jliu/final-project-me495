"""
Determine what robot should do based on computer vision.

Publishers:
    + person_detect (std_msgs/msg/Float32) - Average number of people detected
    in the frame over the last 2 seconds

Subscribers:
    + camera/color/image_raw (sensor_msgs/msg/Image) - Image from the camera

Services:
    + get_characters (polyglotbot_interfaces/srv/GetCharacters) - Returns a
    string of characters from the image
-
"""

# ROS libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge

# Interfaces
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from polyglotbot_interfaces.srv import GetCharacters

# Other libraries
from paddleocr import PaddleOCR
import numpy as np
import cv2
from ultralytics import YOLO


class ComputerVision(Node):
    """Use computer vision to determine handwritten words on a whiteboard."""

    def __init__(self):
        super().__init__("computer_vision")

        # define parameters
        self.dt = 1 / 10.0  # 10 Hz, 30 Hz max

        self.frame = Image().data

        self.bridge = CvBridge()
        self.ocr = PaddleOCR(use_angle_cls=True)
        self.model = YOLO("yolov8n.pt")

        self.ave_num_people = 0.0
        self.num_people = np.zeros(20)

        # Publishers
        self.person_detect = self.create_publisher(
            Float32, "person_detect", QoSProfile(depth=10)
        )

        # Subscribers
        self.get_image = self.create_subscription(Image,
                                                  "camera/color/image_raw",
                                                  self.get_image_callback,
                                                  QoSProfile(depth=10))
        while self.count_publishers("camera/color/image_raw") < 1:
            self.get_logger().info(
                "waiting for camera/color/image_raw publisher")

        # Services
        self.get_characters = self.create_service(GetCharacters,
                                                  "get_characters",
                                                  self.get_characters_callback)

        # Timer
        self.tmr = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        """Control the camera and computer vision."""
        if self.frame is not None:
            temp_frame = self.frame

            results = self.model.predict(source=temp_frame, stream=True,
                                         classes=[0], verbose=False)
            for result in results:
                # self.get_logger().info(f"Results: {result.__len__()}")
                # average num people across 20 frames (2 seconds)
                self.num_people = np.delete(self.num_people, 0)
                self.num_people = np.append(self.num_people, result.__len__())
                self.average_num_people = np.average(self.num_people)

        self.person_detect.publish(Float32(data=self.average_num_people))

    def get_image_callback(self, msg):
        """Get an image from the camera."""
        self.frame = self.bridge.imgmsg_to_cv2(msg)

    def get_characters_callback(self, request, response):
        """Find characters in image and return string."""
        self.temp_frame = self.frame

        cv2.imwrite("node_pic.png", self.temp_frame)
        # cv2.imwrite("filename3.png", self.frame)

        send_results = []
        result = self.ocr.ocr("node_pic.png", cls=True)
        if result[0] is None:
            self.get_logger().info("No text detected")
            send_results = []
        else:
            for idx in range(len(result)):
                res = result[idx]
                for line_num, line in enumerate(res):
                    self.get_logger().info(f"Text: {line}")
                    if line_num == 0:
                        send_results.append(line[1][0])
                    elif line_num == 1:
                        send_results.append(line[1][0])
                    else:
                        send_results[1] = send_results[1] + " " + line[1][0]

        response.words = send_results

        return response


def entry_point(args=None):
    rclpy.init(args=args)
    node = ComputerVision()
    rclpy.spin(node)
    rclpy.shutdown()
