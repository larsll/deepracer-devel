# Copyright (C) 2019-2022 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
FROM ros:humble-ros-base

ARG USERNAME=user-name-goes-here
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    libboost-all-dev \
    python3-pip \
    python3-tk \
    ros-humble-async-web-server-cpp \
    ros-humble-tango-icons-vendor \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-imu-tools \
    ros-humble-tf-transformations \
    ros-humble-cv-bridge \
    ros-humble-robot-localization \    
    ros-humble-rqt-common-plugins \
    ros-humble-pybind11-vendor \
    ros-humble-test-msgs \
    libjsoncpp-dev \
    v4l-utils \
    libx264-dev \
    ffmpeg \
    sudo \
    curl

RUN cd / && curl -O https://aws-deepracer-community-sw.s3.eu-west-1.amazonaws.com/openvino/openvino_2022.3.1_amd64.tgz && \
    tar -xvzf openvino_2022.3.1_amd64.tgz && \
    ln -sf /opt/intel/openvino_2022.3.1 /opt/intel/openvino_2022 && \
    ln -sf /opt/intel/openvino_2022.3.1 /opt/intel/openvino && \
    rm /openvino_2022.3.1_amd64.tgz

RUN pip3 install -U smbus2 pillow autopep8 pyserial rosbags "protobuf<3.21" "pandas<2" "tensorflow-cpu<2.17" "tensorboard" nbconvert ipykernel "numpy<1.25.0" tqdm "networkx>=1.11" "defusedxml>=0.5.0"

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -a -G audio $USERNAME \
    && usermod -a -G video $USERNAME \
    && groupadd --gid 136 i2c \
    && usermod -a -G i2c $USERNAME \
    && usermod -a -G dialout $USERNAME

# Persist history
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=/commandhistory/.bash_history" \
    && mkdir /commandhistory \
    && touch /commandhistory/.bash_history \
    && chown -R $USERNAME /commandhistory \
    && echo $SNIPPET >> "/home/$USERNAME/.bashrc"

RUN mkdir -p /opt/aws/deepracer && ln -sf /workspaces/deepracer-devel/sample-models /opt/aws/deepracer/artifacts

ENTRYPOINT ["/bin/bash", "source /opt/ros/humble/setup.bash"]