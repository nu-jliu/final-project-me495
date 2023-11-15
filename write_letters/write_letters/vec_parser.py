"""
Parser node that parses the all waypoints of a set of letters to a path.

Services
--------
    write [Write]: Write the characters on the board.

Clients
-------
    load_path [Path]: Load the path on robot.

Raises
------
    RuntimeError: Service "load_path" not available.

Returns
-------

"""
from random import random
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup

# messages
from geometry_msgs.msg import Point

# services
from polyglotbot_interfaces.srv import Path, Write


class VecParser(Node):
    def __init__(self):
        super().__init__("vec_parser")
        self.cb_group = ReentrantCallbackGroup()

        self.client_points = self.create_client(
            Path, "load_path", callback_group=self.cb_group
        )

        self.srv_write = self.create_service(
            Write,
            "write",
            callback=self.srv_write_callback,
            callback_group=self.cb_group,
        )

        if not self.client_points.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Service 'load_path' not available")

        # TODO: Make it parameter or get the value from april tags
        self.offset = 0.4
        self.offset_x = 0.1
        self.offset_z = 0.3
        self.offset_letter = 0.03
        self.offset_standup = 0.1
        self.gap_letter = 0.05
        self.scaling = 12.0
        self.x_max = 0.7
        self.x_min = -0.4
        self.z_max = 0.8

        # self.__send_points()

    def srv_write_callback(self, request, response):
        """
        Callback function for the write service.

        Args:
        ----
            request (_type_): _description_
            response (_type_): _description_

        Returns:
        -------
            _type_: _description_
        """
        self.get_logger().info("Writing ...")

        curr_x = -0.55
        curr_z = 0.45
        self.points = []

        for character in request.characters:
            # self.get_logger().info("START")

            max_x = 0.0
            max_y = 0.0
            has_stand_down = False

            for point in character.points:
                # self.get_logger().info(f"{point}")

                px = point.x / self.scaling
                py = point.y / self.scaling

                max_x = max(max_x, px)
                max_y = max(max_y, py)

                x_pos = -(curr_x + px)
                y_pos = -self.offset
                z_pos = py + curr_z

                if not has_stand_down:
                    self.points.append(
                        Point(x=x_pos, y=y_pos + self.offset_standup, z=z_pos)
                    )
                    has_stand_down = True

                self.points.append(Point(x=x_pos, y=y_pos, z=z_pos))

            self.points.append(Point(x=x_pos, y=y_pos + self.offset_standup, z=z_pos))
            curr_x += max_x + self.offset_letter

            if curr_x > 0.25:
                curr_z -= max_y + self.offset_letter
                curr_x = -0.55

                self.get_logger().info("Changing line ...")

            # self.get_logger().info("END")

        self.get_logger().info(f"max x: {curr_x}")
        self.points.append(Point(x=0.3, y=0.0, z=0.5))
        future = self.client_points.call_async(Path.Request(points=self.points))
        # self.get_logger().info(f"{self.points}")
        # rclpy.spin_until_future_complete(self, future)
        future.add_done_callback(self.path_future_callback)

        response.result = "Finished"
        # self.get_logger().info(f"{future.result()}")
        return response

    def path_future_callback(self, future_path: Future):
        self.get_logger().info(f"{future_path.result()}")


def main(args=None):
    rclpy.init(args=args)
    node_parser = VecParser()
    rclpy.spin(node_parser)

    node_parser.destroy_node()
    rclpy.shutdown()