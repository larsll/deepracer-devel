FROM ros:foxy-ros-base

ARG USERNAME=user-name-goes-here
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-all-dev \
    python3-opencv \
    ros-foxy-async-web-server-cpp \
    ros-foxy-vision-opencv \
    ros-foxy-image-transport \
    libjsoncpp-dev \
    v4l-utils \
    ffmpeg \
    sudo

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -a -G audio $USERNAME \
    && usermod -a -G video $USERNAME

ENTRYPOINT ["/bin/bash", "source /opt/ros/foxy/setup.bash"]