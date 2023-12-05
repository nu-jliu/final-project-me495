import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Quaternion, TransformStamped, Point
from enum import Enum, auto
import numpy as np
from geometry_msgs.msg import Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from polyglotbot_interfaces.msg import AprilCoords


class State(Enum):
    WAITING = auto()
    LOOK_UP_TRANSFORM = auto()
    ADD_BOX = auto()


class GetAprilTags(Node):
    """Localizes AprilTags and publishes their coordinates."""

    def __init__(self):
        super().__init__("get_apriltags")

        self.get_logger().info("Starting node ...")

        self.state = State.LOOK_UP_TRANSFORM
        self.cb_group = ReentrantCallbackGroup()

        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

        # Initialize transform information
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Make static transform between camera_color_optical frame and panda_hand
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Camera in the frame of the hand
        hand_camera_tf = TransformStamped()
        hand_camera_tf.header.stamp = self.get_clock().now().to_msg()
        hand_camera_tf.header.frame_id = "panda_link0"
        hand_camera_tf.child_frame_id = "camera_link"

        hand_camera_tf.transform.translation.x = 455e-3
        hand_camera_tf.transform.translation.y = 240e-3
        hand_camera_tf.transform.translation.z = 300e-3

        hand_camera_tf.transform.rotation = Quaternion(
            x=0.0, y=0.0, z=-0.7071068, w=0.7071068
        )

        self.tf_static_broadcaster.sendTransform(hand_camera_tf)

        self.publish_april_coords = self.create_publisher(
            AprilCoords, "april_tag_coords", 10
        )

        self.publish_pen_pos = self.create_publisher(Point, "pen_pos", 10)

        self.top_left_position: Vector3 = None
        self.bottom_left_position: Vector3 = None
        self.bottom_right_position: Vector3 = None

        self.pen_position: Vector3 = None

    # Tag 3 is top left, tag 4 is bottom left, tag 1 is bottom right
    def timer_callback(self):
        """Execute the main timer callback function."""
        # Publish the x,y,z of each AprilTag
        if self.state == State.LOOK_UP_TRANSFORM:
            # TOP LEFT
            try:
                t1 = self.tf_buffer.lookup_transform(
                    "panda_link0",
                    "top_left",
                    rclpy.time.Time(),
                )

                self.position_TL = t1.transform.translation
                self.rotation_TL = t1.transform.rotation
                self.top_left_position = t1.transform.translation

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {"panda_link0"} to {"tag36h11:3"}: {ex}',
                    once=True,
                )
                # return

            # BOTTOM LEFT
            try:
                t2 = self.tf_buffer.lookup_transform(
                    "panda_link0",
                    "bottom_left",
                    rclpy.time.Time(),
                )

                self.position_BL = t2.transform.translation
                self.rotation_BL = t2.transform.rotation

                self.bottom_left_position = t2.transform.translation

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {"panda_link0"} to {"tag36h11:4"}: {ex}',
                    once=True,
                )

            # BOTTOM RIGHT
            try:
                t3 = self.tf_buffer.lookup_transform(
                    "panda_link0",
                    "bottom_right",
                    rclpy.time.Time(),
                )

                self.position_BR = t3.transform.translation
                self.rotation_BR = t3.transform.rotation

                self.bottom_right_position = t3.transform.translation

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {"panda_link0"} to {"tag36h11:1"}: {ex}',
                    once=True,
                )

            try:
                tf_pen = self.tf_buffer.lookup_transform(
                    "panda_link0", "pen_case", rclpy.time.Time()
                )

                self.pen_position = tf_pen.transform.translation

            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform from panda_link0 to pen_case: {ex}",
                    once=True,
                )

            if (
                self.top_left_position
                and self.bottom_left_position
                and self.bottom_right_position
            ):
                msg = AprilCoords()
                msg.p1 = Point(
                    x=self.top_left_position.x,
                    y=self.top_left_position.y,
                    z=self.top_left_position.z,
                )
                msg.p2 = Point(
                    x=self.bottom_left_position.x,
                    y=self.bottom_left_position.y,
                    z=self.bottom_left_position.z,
                )
                msg.p3 = Point(
                    x=self.bottom_right_position.x,
                    y=self.bottom_right_position.y,
                    z=self.bottom_right_position.z,
                )

                self.publish_april_coords.publish(msg)

            if self.pen_position:
                self.publish_pen_pos.publish(
                    Point(
                        x=self.pen_position.x,
                        y=self.pen_position.y,
                        z=self.pen_position.z,
                    )
                )

    def matrix_from_rot_and_trans(self, rotation: Quaternion, translation: Vector3):
        """
        Construct the transformation matrix from rotation and translation.

        Args:
        ----
            rotation (Quaternion): Quaternion object representing the rotation
            translation (Point): Point object representing the

        Returns
        -------
            numpy.ndarray: The resulting homogenous transformation matrix.

        """
        # Extract the values from Q and T
        q0 = rotation.x
        q1 = rotation.y
        q2 = rotation.z
        q3 = rotation.w

        px = translation.x
        py = translation.y
        pz = translation.z

        # First row of the rotation matrix
        r00 = 2.0 * (q0 * q0 + q1 * q1) - 1.0
        r01 = 2.0 * (q1 * q2 - q0 * q3)
        r02 = 2.0 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2.0 * (q1 * q2 + q0 * q3)
        r11 = 2.0 * (q0 * q0 + q2 * q2) - 1.0
        r12 = 2.0 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2.0 * (q1 * q3 - q0 * q2)
        r21 = 2.0 * (q2 * q3 + q0 * q1)
        r22 = 2.0 * (q0 * q0 + q3 * q3) - 1.0

        return np.array(
            [
                [r00, r01, r02, px],
                [r10, r11, r12, py],
                [r20, r21, r22, pz],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )


def get_apriltags_entry(args=None):
    rclpy.init(args=args)
    node = GetAprilTags()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
