# DeepRacer Development - ROS2 Workspace

## Overview

This is a utility repository used to combine all the parts needed to set up a working workspace to develop additional nodes for the AWS DeepRacer that is running ROS2 and Ubuntu 20.04.

## Features

There is a set of different nodes being pulled in as submodules, and the `launch/` directory contains different launch files that combine different combinations of nodes to test out different functionalities.

Some key features:
* Use of onboard IMU sensor
* Creation of video stream and file based on on-car video and IMU
* Image compression end-to-end.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation
Follow these steps to setup the workspace.

### Prerequisites

The `imu_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

1. `geometry_msgs`: This package contains the messages for geometric messages.
1. `sensor_msgs`: This package defines messages for commonly used sensors, including cameras and scanning laser rangefinders.

Additionally the following Python Packages are needed:

1. `pyserial` is needed to allow starting and stopping of recording by serial input.
1. `smbus2` which allows communication via the i2c bus.
1. `BMI160-i2c` which is a driver for the Bosch BMI160 IMU. It is included in `deps/` and will be built from source.

The following access rights are needed if you want to run this as something else than root:
1. `deepracer` to be member of groups `video`, `i2c` and `dialout`.
1. `misc/98-pwm.rules` to be copied to `/etc/udev/rules.d` and `misc/pwm_permissions.sh` to `/opt/aws/deepracer/util/`. (Car only.)

## Downloading and building

### On the DeepRacer

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Initialize all the submodules:

        git submodule update --init --recursive

1. Install the required packages:

        pip3 install smbus2 pyserial
        sudo apt-get install \
                ros-foxy-sensor-msgs \
                ros-foxy-geometry-msgs \
                ros-foxy-async-web-server-cpp \
                ros-foxy-image-transport \
                ros-foxy-compressed-image-transport \
                ros-foxy-imu-tools \
                ros-foxy-tf-transformations \
                ros-foxy-cv-bridge \
                libjsoncpp-dev \
                v4l-utils \
                ffmpeg

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Build the packages:

        colcon build

1. Source the built packages:

        source install/local_setup.bash

### On another computer

The package contains a VS.Code Devcontainer configuration (`.devcontainer/`) which will build a docker container containing the required packages for developing. If the computer in question has a camera connected to `/dev/video?` then the `camera_pkg` will recognize, and use, it.

*Limitations* 
* IMU not available unless a BM160 is connected via USB/I2C.
* LED not available unless a PCA9685 is connected via USB/I2C.

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

