import pyttsx3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from polyglotbot_interfaces.srv import SpeakText


class Speak(Node):
    def __init__(self):
        super().__init__("speaker")

        self.engine = pyttsx3.init()
        self.engine.setProperty("rate", 125)
        self.engine.setProperty("volume", 1.0)

        voices = self.engine.getProperty("voices")
        self.engine.setProperty("voice", voices[1].id)

        self.cb_group = ReentrantCallbackGroup()

        self.srv_speak = self.create_service(
            SpeakText,
            "speak",
            callback=self.srv_speak_callback,
            callback_group=self.cb_group,
        )

    def srv_speak_callback(self, request, response):
        text = request.text

        self.engine.say(f"Translated text: {text}")
        self.engine.runAndWait()
        self.engine.stop()

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node_speaker = Speak()
    rclpy.spin(node=node_speaker)

    node_speaker.destroy_node()
    rclpy.shutdown()
