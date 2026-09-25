FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo \
    python3-colcon-common-extensions \
    ros-humble-ament-cmake-python \
    ros-humble-cv-bridge \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-image-transport \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-pcl-ros \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
CMD ["bash"]
