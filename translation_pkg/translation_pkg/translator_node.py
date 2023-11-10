"""
Test publisher node to see how non-latin characters (e.g. Chinese) fair in msgs with ROS

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Translator(Node):
    """WIP"""

    def __init__(self):
        # Initialize the node
        super().__init__("translator")
        # Initialize variables
        self.init_var()

        #
        # PUBLISHERS
        #
        self.pub_string = self.create_publisher(String, "translated_msg", 10)

        #
        # TIMER
        #
        period = 1 / self.frequency
        self.timer = self.create_timer(period, self.timer_callback)

    def init_var(self):
        """WIP"""
        self.frequency = 1

    #
    # TIMER CALLBACK
    #
    def timer_callback(self):
        """WIP"""
        english_msg = String()
        english_msg.data = "English: Hello my name is Damien"
        self.pub_string.publish(english_msg)

        chinese_msg = String()
        chinese_msg.data = "Chinese: 你好，我的名字是 Damien"
        self.pub_string.publish(chinese_msg)

        french_msg = String()
        french_msg.data = "French: Bonjour, je m'appelle Damien"
        self.pub_string.publish(french_msg)

        hindi_msg = String()
        hindi_msg.data = "Hindi: हैलो मेरा नाम है Damien"
        self.pub_string.publish(hindi_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Translator()
    rclpy.spin(node)
    rclpy.shutdown()
