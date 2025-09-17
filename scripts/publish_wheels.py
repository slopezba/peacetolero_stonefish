#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, Publisher, Timer

from sensor_msgs.msg import JointState


class PropellerPub(Node):

    def __init__(self):
        super().__init__("propeller_node_pub")

        self.publisher: Publisher = self.create_publisher(
            JointState, "/peacetolero/commands/wheel_velocities", 10
        )

        self.msg = JointState()
        self.msg.header.frame_id = "peacetolero/base_link"
        self.msg.name = [
            "peacetolero/wheel_front_right_joint",
            "peacetolero/wheel_front_left_joint",
            "peacetolero/wheel_back_right_joint",
            "peacetolero/wheel_back_left_joint",
        ]
        self.msg.velocity = [1.0, 1.0, 1.0, 1.0]

        self.timer: Timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


if __name__ == "__main__":

    rclpy.init()
    node = PropellerPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
