# OpenVINS (Ubuntu 22.04 + ROS 2 Humble)
FROM docker.io/library/ros:humble-ros-base-jammy

ARG OPENVINS_TAG=v2.7
ARG JOBS=6

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential cmake git python3-colcon-common-extensions \
        libeigen3-dev libboost-all-dev libceres-dev libopencv-dev \
        ros-humble-cv-bridge ros-humble-image-transport ros-humble-rosbag2 \
        ros-humble-rosbag2-storage-default-plugins ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/ws/src && cd /opt/ws/src \
    && git clone --depth 1 -b ${OPENVINS_TAG} https://github.com/rpng/open_vins.git \
    && cd /opt/ws && . /opt/ros/humble/setup.sh \
    && MAKEFLAGS=-j${JOBS} colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release \
       --packages-up-to ov_msckf ov_eval

RUN echo 'source /opt/ws/install/setup.bash' >> /root/.bashrc
WORKDIR /work
