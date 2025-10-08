#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class CmdVelToJoints(Node):
    def __init__(self):
        super().__init__('cmdvel_to_jointstate')

        # Suscriptor a cmd_vel
        self.sub = self.create_subscription(
            Twist,
            '/peacetolero/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Suscriptor a desired_joint_states (para el brazo Alpha)
        self.sub_alpha = self.create_subscription(
            JointState,
            '/peacetolero/alpha/desired_joint_states',
            self.alpha_callback,
            10
        )

        # Publicador de JointState
        self.pub = self.create_publisher(JointState, '/peacetolero/servo_commands', 10)

        # Definir los nombres de las ruedas
        self.wheel_names = [
            'peacetolero/wheel_front_right_joint',
            'peacetolero/wheel_front_left_joint',
            'peacetolero/wheel_back_right_joint',
            'peacetolero/wheel_back_left_joint'
        ]

        # Definir los nombres de los joints del brazo Alpha
        self.alpha_joints = [
            # 'peacetolero/alpha_axis_a',
            'peacetolero/alpha_axis_b',
            'peacetolero/alpha_axis_c',
            'peacetolero/alpha_axis_d',
            'peacetolero/alpha_axis_e'
        ]

        self.max_wheel_speed = 2.0  # rad/s (puedes ajustar según tu modelo)

    def cmd_vel_callback(self, msg: Twist):
        # Escalar de [-1, 1] a [-max_wheel_speed, max_wheel_speed]
        linear = max(-1.5, min(1.0, msg.linear.x)) * self.max_wheel_speed
        angular = max(-1.0, min(1.0, msg.angular.z)) * self.max_wheel_speed

        # Calcular velocidades de ruedas
        v_right = linear + angular
        v_left = linear - angular

        wheel_speeds = [v_right, v_left, v_right, v_left]

        # Construir el JointState
        js = JointState()
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.wheel_names
        js.velocity = wheel_speeds

        self.pub.publish(js)

        self.get_logger().debug(
            f"Published wheel velocities: {wheel_speeds}"
        )

    def alpha_callback(self, msg: JointState):
        """Reenvía los desired_joint_states al mismo tópico de servos en stonefish"""
        # Inicializar todos los joints del Alpha en 0.0
        received = {j: 0.0 for j in self.alpha_joints}

        # Sobrescribir con los valores recibidos (si existen y no NaN)
        for n, v in zip(msg.name, msg.velocity):
            if v != v:  # comprobación de NaN (NaN != NaN en Python)
                v = 0.0
            if n in received:
                received[n] = v

        # Construir la lista ordenada según self.alpha_joints
        velocities = [received[j] for j in self.alpha_joints]

        js = JointState()
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.alpha_joints
        js.velocity = velocities

        self.pub.publish(js)

        self.get_logger().debug(
            f"Published alpha velocities: {dict(zip(self.alpha_joints, velocities))}"
        )



def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
