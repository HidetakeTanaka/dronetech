import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray 

class LandingSubscriber(Node):
    def __init__(self):
        super().__init__('landing_subscriber')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/protoflyer/detected_aruco_markers',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
    
    def listener_callback(self, msg: MarkerArray):
        if msg.markers:
            self.get_logger().info(f"Marker detected! ID(s): { [m.id for m in msg.markers] }")
        else:
            self.get_logger().info("No markers detected")

def main(args=None):
    rclpy.init(args=args)
    node = LandingSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    