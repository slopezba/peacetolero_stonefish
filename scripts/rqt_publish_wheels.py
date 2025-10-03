#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, Publisher, Timer

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped


class WheelPub(Node):

    def __init__(self):
        super().__init__("propeller_node_pub")

        # Subscriber
        self.sub_twist = self.create_subscription(
            TwistStamped,
            "/cmd_vel",  # change if needed
            self.twist_callback,
            10,
        )

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

    def twist_callback(self, cmd: TwistStamped):

        self.msg.header.stamp = self.get_clock().now().to_msg()
        wheel_vel = cmd.twist.linear.x
        self.msg.velocity = [wheel_vel, wheel_vel, wheel_vel, wheel_vel]
        self.get_logger().info(f"Wheels to: ({cmd.twist.linear.x:.2f}) rad/s")
        self.publisher.publish(self.msg)


if __name__ == "__main__":

    rclpy.init()
    node = WheelPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
