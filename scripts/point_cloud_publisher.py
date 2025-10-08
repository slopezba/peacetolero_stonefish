#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Depth -> PointCloud2 for ROS 2 Humble
- Suscribe:  /peacetolero/zedmx/depth_camera/image_depth (sensor_msgs/Image)
- CameraInfo: /peacetolero/zedmx/depth_camera/camera_info (sensor_msgs/CameraInfo)
- Publica:    /peacetolero/zedmx/points (sensor_msgs/PointCloud2)
Soporta depth 32FC1 (m) y 16UC1 (mm, parametrizable con depth_scale).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge

class DepthToPointCloud(Node):
    def __init__(self):
        super().__init__("depth_to_pointcloud")

        # --- Parámetros (todos reconfigurables por CLI/launch) ---
        self.declare_parameter("depth_topic", "/peacetolero/zedmx/depth_camera/image_depth")
        self.declare_parameter("camera_info_topic", "/peacetolero/zedmx/depth_camera/camera_info")
        self.declare_parameter("pointcloud_topic", "/peacetolero/zedmx/points")
        self.declare_parameter("frame_id_override", "")   # si vacío, usa el frame del CameraInfo
        self.declare_parameter("stride", 2)                # decimación de píxeles (1 = sin decimar)
        self.declare_parameter("depth_min", 0.1)           # m
        self.declare_parameter("depth_max", 10.0)          # m
        self.declare_parameter("depth_scale", 0.001)       # para 16UC1: mm->m (0.001); ignorado en 32FC1

        # Lee parámetros
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        self.frame_id_override = self.get_parameter("frame_id_override").get_parameter_value().string_value
        self.stride = int(self.get_parameter("stride").get_parameter_value().integer_value)
        self.depth_min = float(self.get_parameter("depth_min").get_parameter_value().double_value)
        self.depth_max = float(self.get_parameter("depth_max").get_parameter_value().double_value)
        self.depth_scale = float(self.get_parameter("depth_scale").get_parameter_value().double_value)

        # QoS para datos de sensores
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        # CameraInfo suele ser latched; pedimos TRANSIENT_LOCAL si está disponible
        ci_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # Suscripciones y publicador
        self.bridge = CvBridge()
        self.cam_info = None
        self.fx = self.fy = self.cx = self.cy = None
        self.frame_id = None

        self.sub_ci = self.create_subscription(CameraInfo, self.camera_info_topic,
                                               self._camera_info_cb, ci_qos)
        self.sub_depth = self.create_subscription(Image, self.depth_topic,
                                                  self._depth_cb, sensor_qos)
        self.pub_pc = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)

        self.get_logger().info(
            f"[depth_to_pointcloud] depth='{self.depth_topic}', camera_info='{self.camera_info_topic}', "
            f"out='{self.pointcloud_topic}', stride={self.stride}, z∈[{self.depth_min},{self.depth_max}] m"
        )

    # ----------------- Callbacks -----------------
    def _camera_info_cb(self, msg: CameraInfo):
        K = msg.k  # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx, self.fy = float(K[0]), float(K[4])
        self.cx, self.cy = float(K[2]), float(K[5])
        self.cam_info = msg
        self.frame_id = self.frame_id_override if self.frame_id_override else (msg.header.frame_id or "camera_depth_optical_frame")
        self.get_logger().debug(
            f"CameraInfo recibido: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}, frame='{self.frame_id}'"
        )
        # Nos desuscribimos una vez recibido si queremos (opcional)
        # self.destroy_subscription(self.sub_ci)

    def _depth_cb(self, msg: Image):
        if self.cam_info is None or self.fx is None:
            self.get_logger().warn("Esperando CameraInfo para calcular la nube de puntos…", throttle_duration_sec=2.0)
            return

        # Convertir a np.ndarray
        try:
            if msg.encoding in ("32FC1", "32FC"):
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
                depth_m = depth  # ya en metros
            elif msg.encoding in ("16UC1", "mono16"):
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                depth_m = depth.astype(np.float32) * self.depth_scale  # mm->m
            else:
                # Intentar conversión genérica a float32
                depth = self.bridge.imgmsg_to_cv2(msg)
                depth_m = depth.astype(np.float32)
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen de profundidad: {e}")
            return

        h, w = depth_m.shape[:2]
        s = max(1, self.stride)

        # Mallas de pixeles decimados
        uu = np.arange(0, w, s, dtype=np.float32)
        vv = np.arange(0, h, s, dtype=np.float32)
        U, V = np.meshgrid(uu, vv)
        Z = depth_m[::s, ::s]

        # Máscara de válidos (no nan, >0, dentro del rango)
        valid = np.isfinite(Z) & (Z > 0.0) & (Z >= self.depth_min) & (Z <= self.depth_max)
        if not np.any(valid):
            self.get_logger().warn("No hay puntos válidos en este frame tras filtros.", throttle_duration_sec=1.0)
            return

        Zv = Z[valid]
        Uv = U[valid]
        Vv = V[valid]

        # Back-projection (REP-104 camera optical frame: Z adelante, X derecha, Y abajo)
        Xv = (Uv - self.cx) * (Zv / self.fx)
        Yv = (Vv - self.cy) * (Zv / self.fy)

        points = np.stack((Xv, Yv, Zv), axis=-1).astype(np.float32)  # (N,3)

        pc_msg = self._numpy_xyz_to_pointcloud2(points, frame_id=self.frame_id, stamp=msg.header.stamp)
        self.pub_pc.publish(pc_msg)

    # -------------- Utilidad: Numpy -> PointCloud2 --------------
    def _numpy_xyz_to_pointcloud2(self, xyz: np.ndarray, frame_id: str, stamp):
        """
        xyz: (N,3), float32
        """
        assert xyz.ndim == 2 and xyz.shape[1] == 3, "xyz debe ser (N,3)"
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = xyz.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True  # ya filtramos NaNs

        msg.fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * xyz.shape[0]
        msg.data = xyz.tobytes()
        return msg


def main():
    rclpy.init()
    node = DepthToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
