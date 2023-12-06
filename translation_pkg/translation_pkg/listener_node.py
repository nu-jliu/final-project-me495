"""
Node to listen to Translator node for testing.

Subscribers
----
    translated_msg (std_msgs/String) - Contains the translated string of the source string
    target_language (std_msgs/String) - Contains the language code of the target language
        the source string is translated into

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    """Node for listening to translator node."""

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

        self.get_logger().info("Standing by to receive messages...")

    def init_var(self):
        """Initialize all of the listener node's variables."""
        self.frequency = 1

    #
    # SUBSCRIBER CALLBACKS
    #
    def sub_string_callback(self, msg):
        """
        Receive and publishes the translated string to the console.

        Args:
        ----
            msg (std_msgs/String): A message that contains the translated string
                from the translator node

        """
        self.get_logger().info("Translated string: %s" % msg.data)

    def sub_target_language_callback(self, msg):
        """
        Receive and publishes the language code of the target language to the console.

        Args:
        ----
            msg (std_msgs/String): A message that contains the language code
                of the target langauge

        """
        self.get_logger().info("Target language: %s" % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    rclpy.shutdown()
