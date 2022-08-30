#!/usr/bin/env python
import rclpy
from sensor_msgs.msg import Imu

def get_rotation (msg):
    print msg.orientation

rclpy.init_node('test_orientation')

sub = rospy.Subscriber ('/imu/data', Imu, get_rotation)

r = rclpy.Rate(1)
while rclpy.is_ok():
    r.sleep()
