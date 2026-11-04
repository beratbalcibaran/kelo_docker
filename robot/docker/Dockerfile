# KELO ROBILE robot computer (x86_64): platform driver and teleop, ROS 2 Humble.
#
# The robot computer already has this image as kelo_ros2:humble, so this file is needed only to
# rebuild it (from robot/: docker compose -f docker/docker-compose.yml build; internet required).
# The sources are not in the image: they are mounted from robot/src and built in the container.

FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        libboost-thread-dev \
        python3-colcon-common-extensions \
        ros-humble-tf2 \
        ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/kelo_ws
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /root/kelo_ws/install/setup.bash ]; then source /root/kelo_ws/install/setup.bash; fi" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=42" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

CMD ["sleep", "infinity"]
