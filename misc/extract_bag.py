# Copyright (c) 2022 ChenJun

import rosbag2_py
import cv2
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from deepracer_interfaces_pkg.msg import CameraMsg
from rclpy.serialization import deserialize_message


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def main():
    bag_path = 'output/deepracer-bag-20221102-202649'
    storage_options, converter_options = get_rosbag_options(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    storage_filter = rosbag2_py.StorageFilter(topics=['/camera_pkg/video_mjpeg'])
    reader.set_filter(storage_filter)

    bridge = CvBridge()

    count = 0
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg = deserialize_message(data, CameraMsg)

        cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

        path = "output/test"
        if not os.path.exists(path):
            os.makedirs(path)

        cv2.imwrite((path + "/%06i.jpg") % count, cv_img)
        print("Writing image %i" % count)
        count += 1


if __name__ == '__main__':
    main()