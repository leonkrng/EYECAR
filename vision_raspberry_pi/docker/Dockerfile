FROM rwthika/ros2:humble-desktop-full-arm64
 
WORKDIR /app

# Python requirements for pip
COPY docker/requirements.txt .

# Install apt packages
RUN apt-get update && apt-get install -y python3 \
    python3-pip \
    libpcap-dev \
    ffmpeg \
    libsm6 \
    libxext6 \
    libudev-dev \
    python3-gpiozero \
    build-essential \
    cmake \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    python3-dev \
    pkg-config \
    python-is-python3 && \
    pip install --no-cache-dir -r requirements.txt

# Setup opencv with GStreamer support
RUN git config --global http.retry 10
RUN git config --global http.lowSpeedLimit 1
RUN git config --global http.lowSpeedTime 600
RUN git clone --depth 1 https://github.com/opencv/opencv.git /opt/opencv && \
    cd /opt/opencv && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D WITH_GSTREAMER=ON \
          -D BUILD_opencv_python3=ON \
          .. && \
    make -j$(nproc) && \
    make install

# Ros2 lidar driver
 RUN mkdir -p ~/ros2_ws/ && \
    cd ~/ros2_ws/ && \
    git clone https://github.com/Myzhar/ldrobot-lidar-ros2.git src/ldrobot-lidar-ros2 && \
    cd ~/ros2_ws/src/ldrobot-lidar-ros2/scripts/ &&  \
    ./create_udev_rules.sh  


RUN rosdep update && \
    cd ~/ros2_ws/ && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release


COPY . ./release/


RUN chmod +x /app/release/util/docker_entry_script.sh
ENTRYPOINT ["./release/util/docker_entry_script.sh"]

