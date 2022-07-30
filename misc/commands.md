```
ros2 service call /camera_pkg/media_state deepracer_interfaces_pkg/srv/VideoStateSrv "{activate_video: 1}"

ros2 service call /inference_pkg/load_model deepracer_interfaces_pkg/srv/LoadModelSrv "{artifact_path: '/workspaces/deepracer-video/sample-models/Sample_single_cam/model.xml', task_type: 0, pre_process_type: 1, action_space_type: 1}"

ros2 service call /deepracer_navigation_pkg/load_action_space deepracer_interfaces_pkg/srv/LoadModelSrv "{artifact_path: '/workspaces/deepracer-video/sample-models/Sample_single_cam/model_metadata.json', task_type: 0, pre_process_type: 1, action_space_type: 1}"


ros2 service call /inference_pkg/inference_state deepracer_interfaces_pkg/srv/InferenceStateSrv "{start: 1, task_type: 0}"


```

ros2 service call /model_optimizer_pkg/model_optimizer_server deepracer_interfaces_pkg/srv/ModelOptimizeSrv "{model_name: '/workspaces/deepracer-video/sample-models/Sample_single_cam', model_metadata_sensors: [5], training_algorithm: 1, img_format: "BGR", width: 160, height: 120, num_channels: 1, lidar_channels: 0, platform: 0}"
```