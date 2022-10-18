#!/usr/bin/env python3

import logging
import importlib
import sys
import time
import traceback
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.serialization import deserialize_message, serialize_message

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from debug_pkg.constants import (RecordingState)


class BagLogNode(Node):
    """ This node is used to log a topic to a rosbag2.
    """
    _shutdown = Event()
    _target_edit_state = RecordingState.Stopped

    def __init__(self):
        super().__init__('bag_log_node')

        self.declare_parameter(
            'output_path', 'deepracer-bag-{}',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self._output_path = self.get_parameter('output_path').value

        self.declare_parameter(
            'monitor_topic', '/inference_pkg/rl_results',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self._monitor_topic = self.get_parameter('monitor_topic').value

        self.declare_parameter(
            'monitor_topic_type', 'deepracer_interfaces_pkg/msg/InferResultsArray',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self._monitor_topic_type = self.get_parameter('monitor_topic_type').value
        module_name, class_name = self._monitor_topic_type.replace('/', '.').rsplit(".", 1)
        type_module = importlib.import_module(module_name)
        self._monitor_topic_class = getattr(type_module, class_name)

        self.declare_parameter(
            'monitor_topic_timeout', 1,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self._monitor_topic_timeout = self.get_parameter('monitor_topic_timeout').value
        self._monitor_timeout_duration = Duration(seconds=self._monitor_topic_timeout)
        self._monitor_last_received = self.get_clock().now()

    def __enter__(self):

        # Subscription to monitor topic.
        self._main_cbg = ReentrantCallbackGroup()
        self._monitor_node_sub = self.create_subscription(
            self._monitor_topic_class, self._monitor_topic, self._receive_monitor_callback, 1,
            callback_group=self._main_cbg)

        # Check if timeout receiver thread
        self._timeout_check_timer = self.create_timer(
            self._monitor_topic_timeout/10, callback=self._timeout_check_timer_cb, callback_group=self._main_cbg)

        # Change monitor
        self._change_gc = self.create_guard_condition(callback=self._change_cb,
                                                      callback_group=self._main_cbg)

        self.get_logger().info('Monitoring {} of type {} with timeout {} seconds'.format(self._monitor_topic,
                               self._monitor_topic_type, self._monitor_topic_timeout))
        self.get_logger().info('Node started. Ready to control.')

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Stopping the node due to {}.'.format(ExcType.__name__))

        try:
            self._stop_bag()
            self._shutdown.set()
            self._change_gc.destroy()
            self._timeout_check_timer.destroy()
        except:  # noqa E722
            self.get_logger().error("{} occurred.".format(sys.exc_info()[0]))
        finally:
            self.get_logger().info('Node cleanup done. Exiting.')

    def _receive_monitor_callback(self, msg):
        """Permanent method that will receive commands via serial
        """
        try:
            self._monitor_last_received = self.get_clock().now()
            if self._target_edit_state == RecordingState.Stopped:
                self._target_edit_state = RecordingState.Running
                self._change_gc.trigger()
                self.get_logger().info("Got callback from {}. Triggering start.". format(self._monitor_topic))

            elif self._target_edit_state == RecordingState.Running:
                self.get_logger().info(msg.images[0].header.frame_id)
                self._bag_writer.write(self._monitor_topic, serialize_message(msg), self.get_clock().now().nanoseconds)

        except Exception as e:  # noqa E722
            self.get_logger().error("{} occurred in _receive_monitor_callback.".format(sys.exc_info()[0]))

    def _timeout_check_timer_cb(self):
        try:
            dur_since_last_message = self.get_clock().now() - self._monitor_last_received

            if (dur_since_last_message > self._monitor_timeout_duration) and \
                    self._target_edit_state == RecordingState.Running:
                self._target_edit_state = RecordingState.Stopped
                self._change_gc.trigger()
                self.get_logger().info("Timeout. Triggering stop of recording.". format(self._monitor_topic))

        except:  # noqa E722
            self.get_logger().error("{} occurred in _timeout_check_timer_cb.".format(sys.exc_info()[0]))

    def _change_cb(self):
        """Guard condition trigger callback.
        """
        if not self._shutdown.is_set():
            try:
                self.get_logger().info("Changing state to {}"
                                       .format(self._target_edit_state.name))
                if self._target_edit_state == RecordingState.Running:
                    self._start_bag()
                else:
                    self._stop_bag()
            except:  # noqa E722
                self.get_logger().error("{} occurred.".format(sys.exc_info()[0]))

    def _start_bag(self):

        serialization_format = 'cdr'

        if "{}" in self._output_path:
            bag_path = self._output_path.format(time.strftime("%Y%m%d-%H%M%S"))
        else:
            bag_path = self._output_path

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format)

        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')

        self._bag_writer = rosbag2_py.SequentialWriter()
        self._bag_writer.open(storage_options, converter_options)

        self.create_topic(self._bag_writer, self._monitor_topic, self._monitor_topic_type)

    def _stop_bag(self):

        del self._bag_writer

    def create_topic(self, writer, topic_name, topic_type, serialization_format='cdr'):
        """
        Create a new topic.
        :param writer: writer instance
        :param topic_name:
        :param topic_type:
        :param serialization_format:
        :return:
        """
        topic_name = topic_name
        topic = rosbag2_py.TopicMetadata(name=topic_name, type=topic_type,
                                         serialization_format=serialization_format)

        writer.create_topic(topic)

    def test_sequential_writer(self, tmp_path):
        """
        Test for sequential writer.
        :return:
        """
        bag_path = str(tmp_path / 'tmp_write_test')

        storage_options, converter_options = self.get_rosbag_options(bag_path)

        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options, converter_options)

        # create topic
        topic_name = '/chatter'
        self.create_topic(writer, topic_name, 'std_msgs/msg/String')

        for i in range(10):
            msg = String()
            msg.data = f'Hello, world! {str(i)}'
            time_stamp = i * 100

            writer.write(topic_name, serialize_message(msg), time_stamp)

        # close bag and create new storage instance
        del writer


def main(args=None):

    try:
        rclpy.init(args=args)
        with BagLogNode() as bag_log_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(bag_log_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        bag_log_node.destroy_node()
    except KeyboardInterrupt:
        pass
    except:  # noqa: E722
        logging.exception("Error in Node")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
