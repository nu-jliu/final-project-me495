"""
Test listener node to see how non-latin characters (e.g. Chinese) fair in msgs with ROS

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    """WIP"""

    def __init__(self):
        # Initialize the node
        super().__init__("listener")
        # Initialize variables
        self.init_var()

        #
        # SUBSCRIBERS
        #
        self.sub_string = self.create_subscription(
            String, "translated_msg", self.sub_string_callback, 10
        )
        self.sub_string  # Used to prevent warnings

        self.sub_target_language = self.create_subscription(
            String, "target_language", self.sub_target_language_callback, 10
        )
        self.sub_target_language  # Used to prevent warnings

        self.get_logger().info('Standing by to receive messages...')

    def init_var(self):
        """WIP"""
        self.frequency = 1

    #
    # SUBSCRIBER CALLBACKS
    #
    def sub_string_callback(self, msg):
        """WIP"""
        self.get_logger().info('Translated string: %s' % msg.data)

    def sub_target_language_callback(self, msg):
        """WIP"""
        self.get_logger().info('Target language: %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    rclpy.shutdown()