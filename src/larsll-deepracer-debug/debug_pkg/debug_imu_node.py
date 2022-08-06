import rclpy
from rclpy.node import Node


class DebugImuNode(Node):
    def __init__(self):
        super().__init__("debug_imu_node")
        self.get_logger().info("This node just says 'Hello'")


def main(args=None):
    rclpy.init(args=args)
    node = DebugImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
