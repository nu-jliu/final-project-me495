"""
Node that translates a provided input string into a given target language.

Utilizes a version of the Google Translate API to detect the language of a
given source string, receives a specified target language, and then translates that
source string into the target language for subsequent publishing.

Publishers
----------
    translated_msg (std_msgs/String) - Contains the translated string of the source string
    target_language (std_msgs/String) - Contains the language code of the target language
        the source string is translated into

Services
--------
    target_language (polyglotbot_interfaces/TranslateString) - Receives the target language code
        to translate a received source string into
    input_msg (polyglotbot_interfaces/TranslateString) - Receives the source string to then
        translate into a given target language

Parameters
----------
    target_language (string) - Starting target language to translate source string into

"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from polyglotbot_interfaces.srv import TranslateString
import googletrans


class Translator(Node):
    """Node for translating a provided source string."""

    def __init__(self):
        # Initialize the node
        super().__init__("translator")
        # Initialize variables
        self.init_var()

        #
        # PARAMETERS
        #
        self.declare_parameter(
            "target_language",
            "en",
            ParameterDescriptor(
                description="The height between the turtle robot's platform and the ground"
            ),
        )
        self.target_language = (
            self.get_parameter("target_language").get_parameter_value().string_value
        )

        #
        # PUBLISHERS
        #
        self.pub_translated_string = self.create_publisher(String, "translated_msg", 10)
        self.pub_target_language = self.create_publisher(String, "target_language", 10)

        #
        # SERVICES
        #
        self.srv_target_language = self.create_service(
            TranslateString, "target_language", self.target_language_callback
        )
        self.srv_input_string = self.create_service(
            TranslateString, "input_msg", self.input_string_callback
        )

        #
        # TIMER
        #
        period = 1 / self.frequency
        self.timer = self.create_timer(period, self.timer_callback)

    # Initialize variables
    def init_var(self):
        """Initialize all of the translator node's variables."""
        self.frequency = 50
        self.translator = googletrans.Translator()
        self.input_string = None
        self.source_lang = None
        self.translated_string = None

    #
    # TIMER CALLBACK
    #
    def timer_callback(self):
        """
        Timer callback for the translator node.

        In addition to returning the translated string during its service call,
        the node will constantly publish the translated string and its target language
        in case any additional nodes want to use it.
        """
        # If no initial message is received, standby
        if self.input_string is None:
            self.get_logger().info("Standing by to receive input string...", once=True)
        # Once received, begin publishing the translated string
        else:
            msg = String()
            msg.data = self.translated_string
            self.pub_translated_string.publish(msg)
            msg.data = self.target_language
            self.pub_target_language.publish(msg)

    def target_language_callback(self, request, response):
        """
        Change the node's target language.

        When provided with a target language code, the callback will check
        if that code is recognized by the Google Translate API.
        If it is, it will return a string informing the client node that
        the translator node has successfully switched to the target language.
        If not, then the node will inform the client that it is an invald language.

        Args:
        ----
            request (String): Language code of the target language

            response (String): The response object

        Returns
        -------
            String: Contains a string stating the node has successfully switched target langauge

        """
        try:
            test = self.translator.translate(
                text="testing testing", src="en", dest=request.input
            )
            test  # Used to prevent warnings
        # Return specific string to service to signal invalid type
        except Exception as e:
            self.get_logger().warn(f"Failed to set target language: {e}")
            response.output = "INVALID LANGUAGE"
            return response

        self.target_language = request.input

        if self.input_string is not None:
            self.translate_string()
        self.display_strings()

        response.output = "Target language switched to: " + self.target_language
        return response

    def input_string_callback(self, request, response):
        """
        Receive the source string to be translated.

        When provided with a turtle_brick_interfaces/Place message,
        the arena node will switch to a GROUNDED state and update the
        brick's position accordingly

        Args:
        ----
            request (Place): A message that contains the desired x, y,
                and z coordinates of the brick

            response (Empty): The response object

        Returns
        -------
            Empty: Contains nothing

        """
        self.source_lang = self.translator.detect(request.input).lang
        self.input_string = request.input

        self.translate_string()
        self.display_strings()

        response.output = self.translated_string
        return response

    def translate_string(self):
        """Translate current source string into the target language."""
        try:
            self.translated_string = self.translator.translate(
                text=self.input_string, src=self.source_lang, dest=self.target_language
            ).text
        except Exception as e:
            self.get_logger().info("Translation failed")
            self.translated_string = "ERROR: Translation failed"
            e  # Used to prevent warnings

    def display_strings(self):
        """Display the configuration of the node."""
        self.get_logger().info("NEW CONFIGURATION")
        self.get_logger().info("Source language: %s" % self.source_lang)
        self.get_logger().info("Received string: %s" % self.input_string)
        self.get_logger().info("Target language: %s" % self.target_language)
        self.get_logger().info("Translated string: %s" % self.translated_string)


def main(args=None):
    rclpy.init(args=args)
    node = Translator()
    rclpy.spin(node)
    rclpy.shutdown()
