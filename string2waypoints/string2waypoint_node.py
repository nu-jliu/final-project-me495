import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class string2waypoint(Node):
    def __init__(self):
        super().__init__("string2waypoint")

        #Subscribers
        self.sub_translated_string = self.create_subscription(String, 'translated_msg', self.translated_msg_callback, 10 )
        self.sub_target_language = self.create_subscription(String, 'target_language', self.target_language_callback, 10)

        #Publishers
        #publish point message??

        #Service
        #Create service that gets called in the main node?

    def translated_msg_callback(self, msg):
        self.translated_string = msg

    def target_language_callback(self, msg):
        self.target_language = msg

    def string2waypoint(self, translated_string, target_language):
        pass




