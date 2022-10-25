from enum import Enum, IntEnum
import cv2

# Topics / Services
MAIN_CAMERA_TOPIC = "/camera_pkg/display_mjpeg"
VIDEO_STATE_SRV = "/camera_pkg/media_state"
IMU_TOPIC = "/imu_pkg/imu_raw"
PUBLISH_SENSOR_TOPIC = "/sensor_fusion_pkg/sensor_msg"
PUBLISH_VIDEO_TOPIC = "display_stream"
PUBLISH_COMPRESSED_VIDEO_TOPIC = "display_stream/compressed"
STATUS_TOPIC = "status"
RECORDING_STATE_SERVICE_NAME = "recording_state"
LED_SET_SERVICE_NAME = "/servo_pkg/set_led_state"

# Agent Video editor constants
MAX_FRAMES_IN_QUEUE = 2700
KVS_PUBLISH_PERIOD = 1.0/15.0
QUEUE_WAIT_TIME = 1  # In seconds
MONITOR_CHECK_TIME = 0.05

# LED MAX - a bit less than 24 bits
LED_MAX_VALUE = 10000000


class RecordingState(IntEnum):
    """ Status of Recording
    Extends:
        Enum
    """
    Stopped = 0
    Running = 1
    Stopping = 2


class PlaybackState(IntEnum):
    """ Status of Playback
    Extends:
        Enum
    """
    Stopped = 0
    Running = 1
