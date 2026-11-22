# Симуляция полёта БПЛА: ArduPilot SITL + Gazebo Harmonic.
# Камера рендерится через EGL без экрана; для аппаратного рендеринга контейнеру
# пробрасывается /dev/dri (интегрированная графика Intel), иначе -- программный Mesa.
FROM docker.io/library/ubuntu:24.04

ARG JOBS=6
ARG ARDUPILOT_TAG=Copter-4.7.3
ARG ARDUPILOT_GAZEBO_COMMIT=f116b03552d05c8d0b0bed5ed67033a1183121d1
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg lsb-release git build-essential cmake pkg-config \
        python3 python3-pip python3-dev python3-numpy python3-venv \
        libgl1-mesa-dri libegl1 libegl-mesa0 mesa-utils \
        rapidjson-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic python3-gz-transport13 python3-gz-msgs10 \
    && rm -rf /var/lib/apt/lists/*

# ArduPilot SITL (только мультикоптер)
RUN pip3 install --break-system-packages --no-cache-dir \
        empy==3.3.4 future pexpect pymavlink==2.4.47 MAVProxy==1.8.71 opencv-python-headless==4.10.0.84 \
    && git clone --depth 1 --branch ${ARDUPILOT_TAG} --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git /opt/ardupilot \
    && cd /opt/ardupilot && ./waf configure --board sitl && ./waf -j${JOBS} copter

# Плагин связи ArduPilot <-> Gazebo
ENV GZ_VERSION=harmonic
RUN apt-get update && apt-get install -y --no-install-recommends cppzmq-dev libzmq3-dev \
    && rm -rf /var/lib/apt/lists/* \
    && git clone https://github.com/ArduPilot/ardupilot_gazebo.git /opt/ardupilot_gazebo \
    && cd /opt/ardupilot_gazebo && git checkout ${ARDUPILOT_GAZEBO_COMMIT} \
    && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j${JOBS}

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build \
    GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds:/sim/models:/sim/worlds \
    PATH=/opt/ardupilot/build/sitl/bin:/opt/ardupilot/Tools/autotest:$PATH
WORKDIR /sim
