import unittest
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from polyglotbot_interfaces.srv import TranslateString

import rclpy


@pytest.mark.rostest
def generate_test_description():
    my_node = Node(
        package="translation_pkg", executable="translator_node", name="translator_node"
    )

    return (
        LaunchDescription([my_node, ReadyToTest()]),
        {"translator": my_node},
    )


class TestTranslator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Run one time, when the testcase is loaded."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Run one time, when testcase is unloaded."""
        rclpy.shutdown()

    def setUp(self):
        """Run before every test."""
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        """Run after every test."""
        self.node.destroy_node()

    def test_my_publisher(self, launch_service, translator, proc_output):
        """
        Run as a test.

        Args:
        ----
            launch_service: information about the launch
            translator: this was passed in when we created the description
            proc_output: this object streams the output

        """
        # self.my_sub = self.node.create_subscription(
        #     String, "translated_msg", self.sub_callback, 10
        # )
        # self.sub_callback
        self.my_client = self.node.create_client(TranslateString, "input_msg")
        future = self.my_client.call_async(
            TranslateString.Request(text="Hello"))
        future.add_done_callback(self.future_callback)
        # self.starttime = time.time()
        # self.endtime = time.time()
        # self.count = 0
        # self.waittime = 5
        # while (self.endtime - self.starttime) < self.waittime:
        #     rclpy.spin_once(self.node, timeout_sec=self.waittime)
        # hz = self.count / (self.endtime - self.starttime)

        # assert abs(50 - hz) < 5, "Incorrect timer frequency"

    # def sub_callback(self, msg):
    #     self.endtime = time.time()
    #     self.count = self.count + 1

    def future_callback(self, future):
        assert future.result().text is None, "Incorrect translation to None"
