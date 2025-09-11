#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import ManualControlSetpoint

class OnceEcho(Node):
    def __init__(self):
        super().__init__('once_echo_manual_input')
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        # qos.reliability = QoSReliabilityPolicy.BEST_EFFORT       
        qos.durability  = QoSDurabilityPolicy.VOLATILE
        self.sub = self.create_subscription(
            ManualControlSetpoint,
            '/protoflyer/fmu/in/manual_control_input',
            self.cb,
            qos)

    def cb(self, msg: ManualControlSetpoint):
        self.get_logger().info(
            f"roll={msg.roll:.3f}, pitch={msg.pitch:.3f}, yaw={msg.yaw:.3f}, throttle={msg.throttle:.3f}"
        )
        rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(OnceEcho())

if __name__ == "__main__":
    main()
