import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import (Quaternion, Vector3)
from tf_transformations import euler_from_quaternion

class DebugImuNode(Node):
    def __init__(self):
        super().__init__("debug_imu_node")
        self.get_logger().info("Starting IMU Debugging Node")

    def __enter__(self):

        self.get_logger().info("Creating subscription")
        self.imu_sub_cbg = ReentrantCallbackGroup()
        self.imu_sub = self.create_subscription(Imu,
                                 '/imu/data',
                                 self._receive_imu,
                                 5,
                                 callback_group=self.imu_sub_cbg)
    
        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        self.destroy_subscription(self.imu_sub)

    def _receive_imu(self, data: Imu):

        header: Header = data.header
        orientation: Quaternion = data.orientation

        euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], axes='rzyx')
        self.get_logger().info("{:.2f} {:.2f} {:.2f}".format(euler[0], euler[1], euler[2]))


def main(args=None):
    try:
        rclpy.init(args=args)
        with DebugImuNode() as dbg_imu_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(dbg_imu_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        dbg_imu_node.destroy_node()
    except KeyboardInterrupt:
        pass    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
