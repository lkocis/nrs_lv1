FROM ros:jazzy-ros-base

ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git nano iproute2 iputils-ping \
    python3-colcon-common-extensions python3-rosdep python3-numpy \
    python3-yaml python3-pytest libgl1-mesa-dri mesa-utils \
    ros-jazzy-rviz2 ros-jazzy-moveit ros-jazzy-ur \
    ros-jazzy-ur-simulation-gz ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control ros-jazzy-position-controllers \
    ros-jazzy-joint-state-publisher-gui ros-jazzy-tf-transformations \
    ros-jazzy-diagnostic-updater ros-jazzy-rmw-cyclonedds-cpp \
    liburdfdom-tools \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --non-unique --gid ${USER_GID} nrs \
    && useradd --non-unique --uid ${USER_UID} --gid nrs --create-home --shell /bin/bash nrs \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /home/nrs/.bashrc

ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
USER nrs
WORKDIR /workspaces/nrs_lv1
CMD ["bash"]
