#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import MarkerArray


class MarkerToError(Node):
    """
    Subscribe to an ArUco marker topic (MarkerArray),
    publish a boolean flag (visible) and a simple error vector (Vector3).
    """

    def __init__(self):
        super().__init__('marker_to_error')

        # Parameters
        self.declare_parameter('markers_topic', '/protoflyer/detected_aruco_markers')
        self.declare_parameter('visible_topic', '/protoflyer/eolab/precision_landing/visible')
        self.declare_parameter('error_topic', '/protoflyer/eolab/precision_landing/error')
        self.declare_parameter('flip_x', False)   # flip X-axis if needed
        self.declare_parameter('flip_y', False)   # flip Y-axis if needed
        self.declare_parameter('flip_z', False)   # flip Z-axis if needed
        self.declare_parameter('scale', 1.0)      # scale factor for error values

        self.markers_topic = self.get_parameter('markers_topic').get_parameter_value().string_value
        self.visible_topic = self.get_parameter('visible_topic').get_parameter_value().string_value
        self.error_topic   = self.get_parameter('error_topic').get_parameter_value().string_value
        self.flip_x = self.get_parameter('flip_x').get_parameter_value().bool_value
        self.flip_y = self.get_parameter('flip_y').get_parameter_value().bool_value
        self.flip_z = self.get_parameter('flip_z').get_parameter_value().bool_value
        self.scale  = self.get_parameter('scale').get_parameter_value().double_value

        # QoS profiles (match the detector publisher: RELIABLE + VOLATILE)
        sub_qos = QoSProfile(depth=10)
        sub_qos.reliability = QoSReliabilityPolicy.RELIABLE
        sub_qos.durability  = QoSDurabilityPolicy.VOLATILE

        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = QoSReliabilityPolicy.RELIABLE
        pub_qos.durability  = QoSDurabilityPolicy.VOLATILE

        # Publishers
        self.vis_pub = self.create_publisher(Bool, self.visible_topic, pub_qos)
        self.err_pub = self.create_publisher(Vector3, self.error_topic, pub_qos)

        # Subscriber
        self.sub = self.create_subscription(MarkerArray, self.markers_topic, self.callback, sub_qos)

        self.get_logger().info(
            f"[marker_to_error] subscribed: {self.markers_topic}\n"
            f"[marker_to_error] publishing: visible→{self.visible_topic}, error→{self.error_topic}"
        )

        # throttle logs
        self._last_log = self.get_clock().now()

    def callback(self, msg: MarkerArray):
        """
        Callback for MarkerArray messages.
        Takes the first marker's position and converts it into a simple error vector.
        """
        visible = len(msg.markers) > 0
        ex = ey = ez = 0.0

        if visible:
            marker = msg.markers[0]
            ex = float(marker.pose.position.x) * self.scale
            ey = float(marker.pose.position.y) * self.scale
            ez = float(marker.pose.position.z) * self.scale

            if self.flip_x:
                ex = -ex
            if self.flip_y:
                ey = -ey
            if self.flip_z:
                ez = -ez

        # publish results
        self.vis_pub.publish(Bool(data=visible))
        self.err_pub.publish(Vector3(x=ex, y=ey, z=ez))

        # log at 1 Hz
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds * 1e-9 > 1.0:
            self.get_logger().info(f"visible={visible} error=({ex:.2f}, {ey:.2f}, {ez:.2f})")
            self._last_log = now


def main():
    rclpy.init()
    node = MarkerToError()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
