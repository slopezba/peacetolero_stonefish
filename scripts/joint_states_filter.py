#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmJointFilter(Node):
    def __init__(self):
        super().__init__("arm_joint_filter")

        # 🔧 Define qué juntas quieres filtrar (el Alpha en tu caso)
        self.arm_joints = [
            "peacetolero/alpha_axis_a",
            "peacetolero/alpha_axis_b",
            "peacetolero/alpha_axis_c",
            "peacetolero/alpha_axis_d",
            "peacetolero/alpha_axis_e"
        ]

        # Suscribirse al joint_states general del simulador
        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.cb,
            10
        )

        # Publicar solo los del brazo
        self.pub = self.create_publisher(
            JointState,
            "/peacetolero/joint_states",
            10
        )

        self.get_logger().info("ArmJointFilter listo: filtrando solo las juntas del UR5e")

    def cb(self, msg: JointState):
        filtered = JointState()
        filtered.header = msg.header

        for i, name in enumerate(msg.name):
            if name in self.arm_joints:
                filtered.name.append(name)
                filtered.position.append(msg.position[i])
                if msg.velocity:
                    filtered.velocity.append(msg.velocity[i])
                if msg.effort:
                    filtered.effort.append(msg.effort[i])

        if filtered.name:
            self.pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ArmJointFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()