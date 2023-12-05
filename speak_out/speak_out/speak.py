"""
Speak out the translated text.

Subscriptions
-------------
    target_language [String]: The target language to speak out.

Services
--------
    speak [SpeakText]: Speak the translated text.

Returns
-------
    None

"""
import google_speech

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from polyglotbot_interfaces.srv import SpeakText
from std_msgs.msg import String


class Speaker(Node):
    """
    Speak the translated text.

    Args:
    ----
        Node (rclpy.node.Node): Super class of the rclpy Node class.

    Attributes
    ----------
        lang (str): Spoken language.
        cb_group (CallbackGroup): The callback group of the services.

    """

    def __init__(self):
        super().__init__("speaker")

        self.lang = "en"
        self.cb_group = ReentrantCallbackGroup()

        # ---------- Subscriptions ----------
        self.sub_target_lang = self.create_subscription(
            String,
            "target_language",
            callback=self.sub_target_lang_callback,
            qos_profile=10,
        )

        # ---------- Services ----------
        self.srv_speak = self.create_service(
            SpeakText,
            "speak",
            callback=self.srv_speak_callback,
            callback_group=self.cb_group,
        )

    # ---------- Subscription Callbacks ----------
    def sub_target_lang_callback(self, msg: String):
        """
        Get the target language.

        Args:
        ----
            msg (String): Subscribed message for target_language.

        """
        self.lang = msg.data

    # ---------- Service Callbacks ----------
    def srv_speak_callback(self, request, response):
        """
        Speak the translated text.

        Args:
        ----
            request (SpeakText_Request): Request of speak service.
            response (SpeakText_Response): Response of speak service.

        Returns
        -------
            SpeakText_Response: Response of speak service call.

        """
        text = request.text

        google_speech.Speech(text="Translated text", lang="en").play()

        speech = google_speech.Speech(text=text, lang=self.lang)
        speech.play()

        response.success = True
        return response


def main(args=None):
    """
    Start the speaker node.

    Args:
    ----
        args (list[str], optional): Arguments to ros. Defaults to None.

    """
    rclpy.init(args=args)
    node_speaker = Speaker()
    rclpy.spin(node=node_speaker)

    node_speaker.destroy_node()
    rclpy.shutdown()
