FROM ros:jazzy

LABEL maintainer="KELO Team" \
      description="KELO ROS 2 Jazzy: slam_toolbox + AMCL + Nav2 (MPPI omni) + Marvelmind" \
      version="2.0"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
      ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-common \
      ros-jazzy-nav2-simple-commander \
      ros-jazzy-nav2-collision-monitor \
      ros-jazzy-nav2-mppi-controller \
      ros-jazzy-nav2-amcl \
      ros-jazzy-nav2-map-server \
      ros-jazzy-nav2-rviz-plugins \
      ros-jazzy-slam-toolbox \
      ros-jazzy-urg-node \
      ros-jazzy-realsense2-camera \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
      ros-jazzy-rviz2 \
      ros-jazzy-tf2-ros \
      ros-jazzy-tf2-ros-py \
      ros-jazzy-tf2-tools \
      ros-jazzy-diagnostic-updater \
      ros-jazzy-diagnostic-aggregator \
      ros-jazzy-rosbag2 \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-rosidl-default-generators \
      ros-jazzy-launch-xml \
      python3-yaml python3-tk python3-numpy python3-pytest \
      python3-colcon-common-extensions \
      git build-essential ca-certificates curl \
      xvfb x11vnc openbox novnc websockify \
      libgl1-mesa-dri mesa-utils x11-utils \
    && rm -rf /var/lib/apt/lists/*

RUN ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html

WORKDIR /ros2_ws

ARG MM_PKG_SHA=c7eb49bbcfe6937624a98d6a400e5e38d8446b47
ARG MM_MSGS_SHA=9abce079829ade6b1f26d27faf01c0668a3e7459
RUN set -e; mkdir -p /ros2_ws/src && cd /ros2_ws/src \
 && for spec in "marvelmind_ros2_upstream:${MM_PKG_SHA}:marvelmind_ros2" \
                "marvelmind_ros2_msgs_upstream:${MM_MSGS_SHA}:marvelmind_ros2_msgs"; do \
      repo=${spec%%:*}; rest=${spec#*:}; sha=${rest%%:*}; dir=${rest##*:}; \
      mkdir "$dir"; cd "$dir"; git init -q; \
      git remote add origin "https://github.com/MarvelmindRobotics/${repo}.git"; \
      git fetch -q --depth 1 origin "$sha"; git checkout -q FETCH_HEAD; cd ..; \
    done \
 && cd /ros2_ws && . /opt/ros/jazzy/setup.sh \
 && colcon build --packages-select marvelmind_ros2_msgs marvelmind_ros2

ARG RF2O_SHA=b38c68e46387b98845ecbfeb6660292f967a00d3
RUN set -e; cd /ros2_ws/src && mkdir rf2o_laser_odometry && cd rf2o_laser_odometry \
 && git init -q && git remote add origin https://github.com/MAPIRlab/rf2o_laser_odometry.git \
 && git fetch -q --depth 1 origin "${RF2O_SHA}" && git checkout -q FETCH_HEAD \
 && cd /ros2_ws && . /opt/ros/jazzy/setup.sh \
 && colcon build --packages-select rf2o_laser_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY src/kelo_bringup /ros2_ws/src/kelo_bringup
RUN cd /ros2_ws \
 && . /opt/ros/jazzy/setup.sh && . install/setup.sh \
 && colcon build --packages-select kelo_bringup

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/run-stack.sh /run-stack.sh
COPY docker/start-gui.sh /start-gui.sh
COPY docker/cyclonedds.xml /etc/cyclonedds.xml
RUN chmod +x /entrypoint.sh /run-stack.sh /start-gui.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/cyclonedds.xml \
    KELO_DATA_DIR=/data \
    DISPLAY=:99 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1

VOLUME /data

EXPOSE 6080

HEALTHCHECK --interval=30s --timeout=15s --start-period=90s --retries=3 \
  CMD bash -c 'curl -fs http://127.0.0.1:6080/ >/dev/null 2>&1 || (source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list 2>/dev/null | grep -q cmd_vel_gate)'

WORKDIR /ros2_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/start-gui.sh"]
