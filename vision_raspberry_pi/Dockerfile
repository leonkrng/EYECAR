FROM rwthika/ros2:humble-desktop-full-arm64
 
WORKDIR /app

# Python requirements for pip
COPY requirements.txt .

# Install apt packages
RUN apt-get update && apt-get install -y python3 \
    python3-pip \
    libpcap-dev \
    ffmpeg \
    libsm6 \
    libxext6 \
    libudev-dev \
    python3-gpiozero \
    python-is-python3 && \
    pip install --no-cache-dir -r requirements.txt

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

#COPY . /app/release/

RUN chmod +x /app/release/Util/docker_entry_script.sh
ENTRYPOINT ["./release/Util/docker_entry_script.sh"]

