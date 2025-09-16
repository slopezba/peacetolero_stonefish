#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from cola2_msgs.msg import BodyVelocityReq, GoalDescriptor


class VelPub(Node):
    def __init__(self):
        super().__init__("velpub")

        # Create publisher
        self.pub_vel = self.create_publisher(
            BodyVelocityReq,
            "/catamaran/controller/body_velocity_req",
            10,  # queue size
        )

        # Prepare message
        self.vel_req = BodyVelocityReq()
        self.vel_req.header.frame_id = "catamaran/base_link"
        self.vel_req.goal.requester = "sebas"
        self.vel_req.goal.priority = GoalDescriptor.PRIORITY_NORMAL
        self.vel_req.disable_axis.x = False
        self.vel_req.disable_axis.y = True
        self.vel_req.disable_axis.z = True
        self.vel_req.disable_axis.roll = True
        self.vel_req.disable_axis.pitch = True
        self.vel_req.disable_axis.yaw = True
        self.vel_req.twist.linear.x = 5.0
        self.vel_req.twist.linear.y = 0.0
        self.vel_req.twist.linear.z = 0.0
        self.vel_req.twist.angular.z = 0.0

        # Create timer (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.vel_req.header.stamp = self.get_clock().now().to_msg()
        self.pub_vel.publish(self.vel_req)
        self.get_logger().info("Publishing velocity request")


if __name__ == "__main__":
    rclpy.init()
    node = VelPub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("done, stopping")
