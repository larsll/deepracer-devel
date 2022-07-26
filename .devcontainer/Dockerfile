# Copyright (C) 2019-2022 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
FROM ubuntu:20.04 as ov_base

# hadolint ignore=DL3002
USER root

SHELL ["/bin/bash", "-xo", "pipefail", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      curl \
      ca-certificates \
      gnupg \
      python3-minimal \
      python3-setuptools \
      python3-pip && \
      rm -rf /var/lib/apt/lists/*

ARG PUBLIC_KEY="https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB"
ARG APT_REPOSITORY="deb https://apt.repos.intel.com/openvino/2022 focal main"
ARG BUILD_ID=2022.1
# Install full package
RUN curl -O ${PUBLIC_KEY} && \
    apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    echo ${APT_REPOSITORY} | tee /etc/apt/sources.list.d/intel-openvino-2022.list && \
    apt-get update && apt-get install -y --no-install-recommends "openvino-${BUILD_ID}" && \
    rm -rf /var/lib/apt/lists/*

# Install Python and some dependencies for deployment manager
# hadolint ignore=DL3013
RUN pip3 install --no-cache-dir pytest-shutil

# Create CPU only package
RUN mkdir openvino_pkg
RUN /bin/bash -c "source /opt/intel/openvino_2022/setupvars.sh" && \
    python3 /opt/intel/openvino_2022/tools/deployment_manager/deployment_manager.py \
        --targets cpu python3.8 \
        --output_dir openvino_pkg \
        --archive_name openvino_deploy_package

RUN cp -r /opt/intel/openvino_2022/runtime/cmake . && \
    cp -r /opt/intel/openvino_2022/runtime/include . && \
    cp -r /opt/intel/openvino_2022/runtime/lib/intel64/libopenvino_tensorflow_fe.so .

# Replace full package by CPU package
RUN rm -r /opt/intel/ && mkdir -p "/opt/intel/openvino_${BUILD_ID}" && \
    tar -xf /openvino_pkg/openvino_deploy_package.tar.gz -C "/opt/intel/openvino_${BUILD_ID}" && \
    ln --symbolic "/opt/intel/openvino_${BUILD_ID}" /opt/intel/openvino_2022 && \
    ln --symbolic "/opt/intel/openvino_${BUILD_ID}" /opt/intel/openvino && \
    mv /cmake /opt/intel/openvino_2022/runtime/cmake && \
    mv /include /opt/intel/openvino_2022/runtime/include && \
    mv libopenvino_tensorflow_fe.so /opt/intel/openvino_2022/runtime/lib/intel64

FROM ros:foxy-ros-base

ARG USERNAME=user-name-goes-here
ARG USER_UID=1000
ARG USER_GID=$USER_UID

COPY --from=ov_base /opt/intel /opt/intel

RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-all-dev \
    python3-opencv \
    python3-pip \
    ros-foxy-async-web-server-cpp \
    ros-foxy-vision-opencv \
    ros-foxy-image-transport \
    libjsoncpp-dev \
    v4l-utils \
    ffmpeg \
    sudo

RUN pip3 install -U smbus2 pillow

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -a -G audio $USERNAME \
    && usermod -a -G video $USERNAME \
    && groupadd --gid 136 i2c \
    && usermod -a -G i2c $USERNAME

# Persist history
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=/commandhistory/.bash_history" \
    && mkdir /commandhistory \
    && touch /commandhistory/.bash_history \
    && chown -R $USERNAME /commandhistory \
    && echo $SNIPPET >> "/home/$USERNAME/.bashrc"

ENTRYPOINT ["/bin/bash", "source /opt/ros/foxy/setup.bash"]