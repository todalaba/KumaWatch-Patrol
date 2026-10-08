# KumaWatch Patrol: ROS 2 Humble + Gazebo Classic 11 + Navigation2 + TurtleBot3
#
#   docker build -t kumawatch_patrol -f docker/Dockerfile .
#   ./docker/run.sh            # X11 転送付きで起動(Linux)
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    TURTLEBOT3_MODEL=waffle \
    ROS_DOMAIN_ID=30

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-turtlebot3 \
      ros-humble-turtlebot3-gazebo \
      ros-humble-turtlebot3-msgs \
      python3-colcon-common-extensions \
      python3-numpy \
      python3-opencv \
      ffmpeg \
      python3-matplotlib \
      python3-pytest \
      python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src /ws/src
COPY scripts /ws/scripts
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo 'source /usr/share/gazebo/setup.sh' >> /root/.bashrc \
 && echo 'source /ws/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
