"""
Combines individual packages with control from a single node.

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
import numpy as np


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    TBD = auto()  # To be determined


class Polyglotbot(Node):
    """Combines individual packages with control from a single node."""

    def __init__(self):
        super().__init__("polyglotbot")

        # define parameters
        self.dt = 1/100.0  # 100 Hz

        # Timer
        self.tmr = self.create_timer(self.dt, self.timer_callback)

        # self.setup_camera()

    def timer_callback(self):
        """Control the Franka."""
        pass


def entry_point(args=None):
    rclpy.init(args=args)
    node = Polyglotbot()
    rclpy.spin(node)
    rclpy.shutdown()
