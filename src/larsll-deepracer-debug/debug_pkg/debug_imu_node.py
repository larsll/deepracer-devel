import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import (Quaternion, AccelWithCovarianceStamped)
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class DebugImuNode(Node):
    def __init__(self):
        super().__init__("debug_imu_node")
        self.get_logger().info("Starting IMU Debugging Node")
        self.last_time = None
        self.velocity = 0.0
        self.prev_acceleration = 0.0
        self.msg = 0
        self.accl_max = 0.0
        self.accl_min = 0.0

    def __enter__(self):

        self.get_logger().info("Creating subscription")
        self.imu_sub_cbg = ReentrantCallbackGroup()
        self.imu_sub = self.create_subscription(Imu,
                                                '/imu/data',
                                                self._receive_imu,
                                                5,
                                                callback_group=self.imu_sub_cbg)

        self.accel_sub_cbg = ReentrantCallbackGroup()
        self.accel_sub = self.create_subscription(AccelWithCovarianceStamped,
                                                  '/accel/filtered_x',
                                                  self._receive_accel,
                                                  5,
                                                  callback_group=self.accel_sub_cbg)

        self.odom_sub_cbg = ReentrantCallbackGroup()
        self.odom_sub = self.create_subscription(Odometry,
                                                 '/odometry/filtered_x',
                                                 self._receive_odom,
                                                 5,
                                                 callback_group=self.odom_sub_cbg)

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        self.destroy_subscription(self.imu_sub)

    def _receive_imu(self, data: Imu):

        self.msg += 1

        header: Header = data.header
        time: Time = Time(seconds=header.stamp.sec, nanoseconds=header.stamp.nanosec)

        if self.last_time is None:
            self.last_time = time
        else:
            delta_time: Duration = time - self.last_time
            delta_time_ms = delta_time.nanoseconds / 10 ** 6

            self.last_time = time
            self.prev_acceleration = data.linear_acceleration.x

            if self.msg > 100:
                self.accl_max = max(self.accl_max, data.linear_acceleration.x)
                self.accl_min = min(self.accl_min, data.linear_acceleration.x)

        orientation: Quaternion = data.orientation
        self.velocity += data.linear_acceleration.x * delta_time_ms / (10 ** 3)

        if (self.msg % 10 == 0):
            euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], axes='rzyx')
            self.get_logger().info("{:.2f} {:.2f} {:.2f} {:+.0f} {:+.0f} {:+.0f}".format(self.velocity,
                                                                                         self.accl_min, self.accl_max,
                                                                                         euler[0] * 180 / math.pi,
                                                                                         euler[1] * 180 / math.pi,
                                                                                         euler[2] * 180 / math.pi))

    def _receive_accel(self, data: AccelWithCovarianceStamped):

        self.msg += 1

        header: Header = data.header
        time: Time = Time(seconds=header.stamp.sec, nanoseconds=header.stamp.nanosec)

        if self.last_time is None:
            self.last_time = time
        else:
            delta_time: Duration = time - self.last_time
            delta_time_ms = delta_time.nanoseconds / 10 ** 6
            avg_acceleration = (self.prev_acceleration + data.accel.accel.linear.x) / 2

            self.last_time = time
            self.prev_acceleration = data.accel.accel.linear.x

        # orientation: Quaternion = data.orientation
        self.velocity += avg_acceleration * delta_time_ms / (10 ** 3)

        if (self.msg % 5 == 0):
            # euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], axes='rzyx')
            self.get_logger().info("{:.2f}".format(self.velocity))

    def _receive_odom(self, data: Odometry):

        self.msg += 1

        speed: float = (data.twist.twist.linear.x ** 2 + data.twist.twist.linear.y **
                        2 + data.twist.twist.linear.z ** 2) ** 0.5

        if (self.msg % 5 == 0):
            self.get_logger().info("{:.3f}".format(speed))


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
