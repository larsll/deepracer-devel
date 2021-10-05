# DeepRacer Video Recording - ROS2 Workspace

## Overview

This is a utility repository used to combine all the parts needed to set up a working workspace to take onboard video from the AWS DeepRacer that is running ROS2 and Ubuntu 20.04.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation
Follow these steps to setup the workspace.

### Prerequisites

The `imu_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

1. `geometry_msgs`: This package contains the messages for geometric messages.
1. `sensor_msgs`: This package defines messages for commonly used sensors, including cameras and scanning laser rangefinders.

Additionally the following Python Packages are needed:

1. `smbus2` which allows communication via the i2c bus.
1. `BMI160-i2c` which is a driver for the Bosch BMI160 IMU. We need to build from source.

The following access rights are needed if you want to run this as something else than root:
1. `deepracer` to be member of groups `video` and `i2c`

## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Initialize all the submodules:

        git submodule update --init --recursive

1. Install the required packages:

        pip3 install smbus2
        sudo apt-get install ros-foxy-sensor-msgs ros-foxy-geometry-msgs

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Build the packages:

        colcon build

1. Source the built packages:

        source install/local_setup.bash


## Run the stack

To launch the built stack on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands:

1. Stop DeepRacer core

        sudo systemctl stop deepracer-core  

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source install/local_setup.bash

1. Launch the stack using the launch script:

        ros2 launch record.launch 

## Launch files

The `record.launch`, included in this package, provides an example demonstrating how to launch the full video recording stack independently from the DeepRacer application.

        <launch >
        <arg name="output" default="output/deepracer-{}.mp4" />
        <node pkg="camera_pkg" exec="camera_node" output="log">
            <param name="downscale_images" type="int" value="2" /> 
            <param name="display_topic_enable" type="bool" value="False" /> 
        </node>
        <node pkg="imu_pkg" exec="imu_node" output="log" />
        <node pkg="incar_video_pkg" exec="incar_video_capture_node" output="log">
            <param name="duplicate_frame" type="bool" value="True" /> 
        </node>
        <executable cmd="ros2 bag record /imu_msg/raw" output="screen" cwd="./output" />
        <node pkg="incar_video_pkg" exec="incar_video_edit_node" output="log">
            <param name="racecar_name" type="string" value="Duckworth" /> 
            <param name="output_file_name" type="string" value="$(var output)" />
        </node>
        </launch>


