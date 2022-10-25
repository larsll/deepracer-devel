#!/usr/bin/env python3

import logging
import traceback
import sys
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from sensor_msgs.msg import Image as ROSImg
from sensor_msgs.msg import CompressedImage as ROSCImg

from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr

from deepracer_interfaces_pkg.msg import CameraMsg
from deepracer_interfaces_pkg.srv import VideoStateSrv

from debug_pkg.constants import PlaybackState

CAMERA_MSG_TOPIC = "video_mjpeg"
DISPLAY_MSG_TOPIC = "display_mjpeg"
ACTIVATE_CAMERA_SERVICE_NAME = "media_state"
DEFAULT_IMAGE_WIDTH = 640
DEFAULT_IMAGE_HEIGHT = 480


class CameraReplayNode(Node):
    """ This node is used play back ROS1 bags (created from the Robomaker container) into a ROS2 stack.
    """

    _play_state = PlaybackState.Stopped
    _play_messages_generator = None
    _playback_frames = 0

    def __init__(self):
        super().__init__('camera_replay_node')

        self.declare_parameter('resize_images', False, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('resize_images_factor', 4, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('fps', 15, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('display_topic_enable', True, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('input_file', "", ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING))

        self._resize_images = self.get_parameter('resize_images').value
        self._resize_images_factor = self.get_parameter('resize_images_factor').value
        self._display_topic_enable = self.get_parameter('display_topic_enable').value
        self._fps = self.get_parameter('fps').value
        self._input_file = self.get_parameter('input_file').value

        # Init cv bridge
        self._bridge = CvBridge()

    def __enter__(self):

        self._main_cbg = ReentrantCallbackGroup()

        # Call ROS service to enable the Video Stream
        self._camera_state_srv = self.create_service(VideoStateSrv, ACTIVATE_CAMERA_SERVICE_NAME,
                                                     callback=self._state_service_callback,
                                                     callback_group=self._main_cbg)

        # Publisher to broadcast the video stream.
        self._camera_pub = self.create_publisher(CameraMsg, CAMERA_MSG_TOPIC, 1,
                                                 callback_group=self._main_cbg)
        self._display_pub = self.create_publisher(ROSImg, DISPLAY_MSG_TOPIC, 250,
                                                  callback_group=self._main_cbg)
        self._display_cpub = self.create_publisher(ROSCImg, DISPLAY_MSG_TOPIC + "/compressed", 250,
                                                   callback_group=self._main_cbg)

        self._playback_timer = None

        self.get_logger().info('Node started. Ready to start playback.')

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Stopping the node due to {}.'
                               .format(ExcType.__name__))
        if self._play_state != PlaybackState.Stopped:
            self._play_state = PlaybackState.Stopped
            self._stop_playback()
            self.destroy_timer(self._playback_timer)

        self.get_logger().info('Node cleanup done. Exiting.')

    def _start_playback(self):
        """ Method that is used to start the playback.
        """
        self._reader = Reader(self._input_file)
        self._reader.open()
        self.get_logger().info("Reading {}. Messages: {}. Available topics:".format(self._input_file,
                                                                                    self._reader.message_count))
        for connection in self._reader.connections:
            self.get_logger().info("    {}, {}".format(connection.topic, connection.msgtype))

        self._play_state = PlaybackState.Running
        self._playback_frames = 0
        self._play_messages_generator = self._reader.messages()

        # Prepare timer
        self._playback_timer = self.create_timer(1.0/(self._fps), self._playback_timer_cb,
                                                 callback_group=self._main_cbg)

    def _stop_playback(self):
        """ Method that is used to stop the playback.
        """
        self.get_logger().info('Stopping the playback after {} frames.'.format(self._playback_frames))
        self._play_state = PlaybackState.Stopped

        if self._reader is not None:
            self._reader.close()

        # Stop timer
        self._playback_timer.destroy()

    def _playback_timer_cb(self):

        # Play next message

        try:
            connection, timestamp, rawdata = next(self._play_messages_generator)
            msg: ROSImg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
            img_cv2 = self._bridge.imgmsg_to_cv2(msg, "rgb8")
            img_cv2 = cv2.cvtColor(img_cv2, cv2.COLOR_RGB2BGR)

            if (self._resize_images):
                img_cv2 = cv2.resize(img_cv2, dsize=(int(DEFAULT_IMAGE_WIDTH / self._resize_images_factor),
                                     int(DEFAULT_IMAGE_HEIGHT / self._resize_images_factor)))

            c_msg = self._bridge.cv2_to_compressed_imgmsg(img_cv2)
            c_msg.format = "bgr8; jpeg compressed bgr8"

            camera_msg: CameraMsg = CameraMsg()
            camera_msg.images.append(c_msg)

            self._camera_pub.publish(camera_msg)
            if self._display_topic_enable:
                self._display_pub.publish(self._bridge.cv2_to_imgmsg(img_cv2, "bgr8"))
                self._display_cpub.publish(c_msg)

            self._playback_frames += 1

        except StopIteration:
            self.get_logger().info("End of stream after {} messages.".format(self._playback_frames))
            self._stop_playback()
        except:  # noqa E722
            self.get_logger().error("{} occurred.".format(str(sys.exc_info())))
            self._stop_playback()

    def _state_service_callback(self, req, res):
        """Callback for the playback state service.
        Args:
            req (VideoStateSrv.Request): Request change to the playback state
            res (VideoStateSrv.Response): Response object with error(int) flag
                                           to indicate if the service call was
                                           successful.

        Returns:
            VideoStateSrv.Response: Response object with error(int) flag to
                                     indicate if the call was successful.
        """
        if self._play_state == PlaybackState.Running and req.activate_video == 0:
            self._stop_playback()
            res.error = 0

        elif (self._play_state == PlaybackState.Running) and (req.activate_video == 1):
            res.error = 1

        elif self._play_state == PlaybackState.Stopped and req.activate_video == 0:
            res.error = 0

        elif self._play_state == PlaybackState.Stopped and req.activate_video == 1:
            self._start_playback()
            res.error = 0

        return res


def main(args=None):

    try:
        rclpy.init(args=args)
        with CameraReplayNode() as camera_replay_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(camera_replay_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        camera_replay_node.destroy_node()
    except KeyboardInterrupt:
        pass
    except:  # noqa: E722
        logging.exception("Error in Node")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
