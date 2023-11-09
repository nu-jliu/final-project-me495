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

        # Services
        self.get_image = self.create_service(String, "get_image", self.get_image_callback)

        # Timer
        self.tmr = self.create_timer(self.dt, self.timer_callback)

        self.setup_camera()

    def timer_callback(self):
        """Control the camera and computer vision."""
        pass

    def get_image_callback(self, request, response):
        """Return an image."""

        frameset = self.pipeline.wait_for_frames()

        color_frame = frameset.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        cv2.imwrite("filename.png", color_image)

        reader = easyocr.Reader(['es', 'en', 'de', 'fr'], gpu=False, detail=0)  # this needs to run only once to load the model into memory
        results = reader.readtext('filename.png')

        for result in results:
            self.get_logger().info(f"Text: {result}")

        response = String()
        response.data = "Hello World"

        return response

    def setup_camera(self):
        # Create a pipeline
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if self.device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)


def entry_point(args=None):
    rclpy.init(args=args)
    node = ComputerVision()
    rclpy.spin(node)
    rclpy.shutdown()
