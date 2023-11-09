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

# Interfaces
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image

# Other libraries
import math
from enum import Enum, auto
import easyocr
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

        # Subscribers
        self.get_image = self.create_subscription(Image, "/camera/color/image_raw", self.get_image_callback, QoSProfile(depth=10))

        # Services
        self.get_string = self.create_service(String, "get_string", self.get_string_callback)

        # Timer
        self.tmr = self.create_timer(self.dt, self.timer_callback)

        # self.setup_camera()

    def timer_callback(self):
        """Control the camera and computer vision."""
        pass

    def get_image_callback(self, msg):
        """Get an image from the camera."""
        self.frame = msg.data

    def get_string_callback(self, request, response):
        """Find characters in image and return string."""

        cv2.imwrite("filename3.png", self.frame)

        # reader = easyocr.Reader(['es', 'en', 'de', 'fr'], gpu=False, detail=0)  # this needs to run only once to load the model into memory
        # results = reader.readtext('filename3.png')

        # for result in results:
        #     self.get_logger().info(f"Text: {result}")

        response = String()
        response.data = "Hello World"

        return response


def entry_point(args=None):
    rclpy.init(args=args)
    node = ComputerVision()
    rclpy.spin(node)
    rclpy.shutdown()
