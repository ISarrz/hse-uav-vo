# VINS-Fusion (Ubuntu 20.04 + ROS Noetic)
FROM docker.io/library/ros:noetic-ros-base-focal

ARG VINS_COMMIT=be55a937a57436548ddfb1bd324bc1e9a9e828e0
ARG JOBS=6

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential cmake git python3-catkin-tools \
        libeigen3-dev libceres-dev libopencv-dev \
        ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf ros-noetic-rosbag \
    && rm -rf /var/lib/apt/lists/*

COPY patches/vins_opencv4.sh /tmp/vins_opencv4.sh

RUN mkdir -p /opt/catkin_ws/src && cd /opt/catkin_ws/src \
    && git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git \
    && cd VINS-Fusion && git checkout ${VINS_COMMIT} \
    && bash /tmp/vins_opencv4.sh /opt/catkin_ws/src/VINS-Fusion \
    && cd /opt/catkin_ws && . /opt/ros/noetic/setup.sh \
    && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build -j${JOBS}

RUN echo 'source /opt/catkin_ws/devel/setup.bash' >> /root/.bashrc
WORKDIR /work
