# ORB-SLAM3 (Ubuntu 20.04 + ROS Noetic)
FROM docker.io/library/ros:noetic-ros-base-focal

ARG ORBSLAM3_COMMIT=4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
ARG PANGOLIN_TAG=v0.6
ARG JOBS=6

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential cmake git wget unzip pkg-config \
        libeigen3-dev libopencv-dev libboost-all-dev libssl-dev \
        libglew-dev libgl1-mesa-dev libepoxy-dev libpython3-dev \
    && rm -rf /var/lib/apt/lists/*

# Pangolin нужен ORB-SLAM3 даже без визуализации
RUN git clone --depth 1 -b ${PANGOLIN_TAG} https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin \
    && cmake -S /opt/Pangolin -B /opt/Pangolin/build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF \
    && cmake --build /opt/Pangolin/build -j${JOBS} && cmake --install /opt/Pangolin/build && ldconfig

RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /opt/ORB_SLAM3 \
    && cd /opt/ORB_SLAM3 && git checkout ${ORBSLAM3_COMMIT} \
    # в Ubuntu 20.04 OpenCV 4.2, а CMakeLists требует 4.4
    && sed -i 's/find_package(OpenCV 4.4)/find_package(OpenCV 4.2)/' CMakeLists.txt \
    && sed -i "s/make -j[0-9]*/make -j${JOBS}/g; s/make -j$/make -j${JOBS}/g" build.sh \
    && ./build.sh

# в контейнере нет дисплея: отключаем окно Pangolin в примерах и пересобираем их
RUN cd /opt/ORB_SLAM3 \
    && sed -i -E 's/(ORB_SLAM3::System SLAM\(.*), *true\);/\1, false);/' Examples/*/*.cc \
    && cd build && make -j${JOBS}

ENV ORBSLAM3_ROOT=/opt/ORB_SLAM3
WORKDIR /work
