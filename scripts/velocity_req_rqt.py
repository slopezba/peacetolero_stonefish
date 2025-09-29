#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from cola2_msgs.msg import BodyVelocityReq, GoalDescriptor


class TwistToVelReq(Node):
    def __init__(self):
        super().__init__("twist_to_velreq")

        # Publisher
        self.pub_vel = self.create_publisher(
            BodyVelocityReq,
            "/catamaran/controller/body_velocity_req",
            10,
        )

        # Subscriber
        self.sub_twist = self.create_subscription(
            Twist,
            "/cmd_vel",  # change if needed
            self.twist_callback,
            10,
        )

        # Prepare a base BodyVelocityReq template
        self.vel_req = BodyVelocityReq()
        self.vel_req.header.frame_id = "catamaran/base_link"
        self.vel_req.goal.requester = "sebas"
        self.vel_req.goal.priority = GoalDescriptor.PRIORITY_NORMAL
        self.vel_req.disable_axis.x = False
        self.vel_req.disable_axis.y = True
        self.vel_req.disable_axis.z = True
        self.vel_req.disable_axis.roll = True
        self.vel_req.disable_axis.pitch = True
        self.vel_req.disable_axis.yaw = False

    def twist_callback(self, msg: Twist):
        # Fill in BodyVelocityReq with Twist data
        self.vel_req.header.stamp = self.get_clock().now().to_msg()

        self.vel_req.twist.linear.x = msg.linear.x
        self.vel_req.twist.linear.y = msg.linear.y
        self.vel_req.twist.linear.z = msg.linear.z

        self.vel_req.twist.angular.x = msg.angular.x
        self.vel_req.twist.angular.y = msg.angular.y
        self.vel_req.twist.angular.z = msg.angular.z

        # Publish converted message
        self.pub_vel.publish(self.vel_req)
        self.get_logger().info(
            f"Converted Twist to BodyVelocityReq: lin=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}), "
            f"ang=({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwistToVelReq()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("done, stopping")


if __name__ == "__main__":
    main()
