#!/bin/bash
ros2 launch launch/optimize.launch

ros2 service call /model_optimizer_pkg/model_optimizer_server deepracer_interfaces_pkg/srv/ModelOptimizeSrv "{Sample_single_cam/model', model_metadata_sensors: [1], training_algorithm: 1, img_format: "BGR", width: 160, height: 120, num_channels: 1, lidar_channels: 0, platform: 0}"