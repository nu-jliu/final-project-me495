"""
Determine what robot should do based on computer vision.

Publishers:
  + abc (type) - description

Services:
  + abc (type) - description

Parameter
  + abc (type) - description
-
"""

# ROS libraries
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_ros.transform_listener import TransformListener
from cv_bridge import CvBridge

# Interfaces
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CompressedImage
from polyglotbot_interfaces.srv import GetCharacters

# Other libraries
import math
from enum import Enum, auto
from paddleocr import PaddleOCR
import pyrealsense2 as rs
import numpy as np
import cv2


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    TBD = auto()  # To be determined


class ComputerVision(Node):
    """USe computer vision to determine handwritten words on a whiteboard."""

    def __init__(self):
        super().__init__("computer_vision")

        # define parameters
        self.dt = 1/100.0  # 100 Hz

        self.frame = Image().data
        # self.temp_frame = np.asanyarray(cv2.imread("filename.png"))
        # self.get_logger().info(f"Got image: {self.temp_frame}")
        # print(self.temp_frame)

        self.bridge = CvBridge()
        self.ocr = PaddleOCR(use_angle_cls=True)

        # Subscribers
        self.get_image = self.create_subscription(Image, "/camera/color/image_raw", self.get_image_callback, QoSProfile(depth=10))

        # Services
        self.get_characters = self.create_service(GetCharacters, "get_characters", self.get_characters_callback)

        # Timer
        self.tmr = self.create_timer(self.dt, self.timer_callback)

        # self.setup_camera()

    def timer_callback(self):
        """Control the camera and computer vision."""
        pass

    def get_image_callback(self, msg):
        """Get an image from the camera."""
        self.frame = self.bridge.imgmsg_to_cv2(msg)
        # self.frame = np.asanyarray(msg.data)
        # self.get_logger().info(f"Got image: {self.frame}")

    def get_characters_callback(self, request, response):
        """Find characters in image and return string."""

        self.temp_frame = self.frame

        cv2.imwrite("filename3.png", self.temp_frame)
        # cv2.imwrite("filename3.png", self.frame)

        # reader = easyocr.Reader(['es', 'en', 'de', 'fr'], gpu=False)  # this needs to run only once to load the model into memory
        # results = reader.readtext(self.temp_frame, detail=0)

        result = self.ocr.ocr('filename3.png', cls=True)
        for idx in range(len(result)):
            res = result[idx]
            for line in res:
                self.get_logger().info(f"Text: {line}")

        # for result in results:
        #     self.get_logger().info(f"Text: {result}")

        response.words = ["hello", "world"]

        return response


def entry_point(args=None):
    rclpy.init(args=args)
    node = ComputerVision()
    rclpy.spin(node)
    rclpy.shutdown()
