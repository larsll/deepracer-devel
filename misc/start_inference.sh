#!/bin/bash
ros2 service call /inference_pkg/load_model deepracer_interfaces_pkg/srv/LoadModelSrv "{artifact_path: '$1/model.tflite', task_type: 0, pre_process_type: 1, action_space_type: 1}"

ros2 service call /deepracer_navigation_pkg/load_action_space deepracer_interfaces_pkg/srv/LoadModelSrv "{artifact_path: '$1/model_metadata.json', task_type: 0, pre_process_type: 1, action_space_type: 1}"

ros2 service call /camera_pkg/media_state deepracer_interfaces_pkg/srv/VideoStateSrv "{activate_video: 1}"

# ros2 service call /inference_pkg/inference_state deepracer_interfaces_pkg/srv/InferenceStateSrv "{start: 1, task_type: 0}"