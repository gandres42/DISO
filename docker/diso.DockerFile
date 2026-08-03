FROM osrf/ros:lyrical-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# Build dependencies
#
# Everything DISO needs is packaged for ROS 2 lyrical / Ubuntu 26.04, so unlike
# the original Noetic image nothing has to be built from source.
#   ros-lyrical-libg2o   - graph optimisation back end
#   ros-lyrical-sophus   - Lie group maths used by the odometry edges
#   libjsoncpp-dev       - Ubuntu 26.04's VTK 9.5 CMake targets (pulled in via PCL,
#                          which pcl_conversions requires) reference JsonCpp::JsonCpp
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git ninja-build vim \
      python3-colcon-common-extensions python3-rosdep \
      ros-lyrical-rmw-zenoh-cpp \
      ros-lyrical-libg2o \
      ros-lyrical-sophus \
      ros-lyrical-cv-bridge \
      ros-lyrical-image-transport \
      ros-lyrical-image-transport-plugins \
      ros-lyrical-message-filters \
      ros-lyrical-pcl-conversions \
      ros-lyrical-tf2-ros \
      ros-lyrical-rosbag2-storage-mcap \
      ros-lyrical-rviz2 \
      libeigen3-dev libopencv-dev libpcl-dev libjsoncpp-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Workspace
# ---------------------------------------------------------------------------
ARG USERNAME=diso
RUN useradd -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER ${USERNAME}

WORKDIR /home/${USERNAME}/diso_ws/src
RUN git clone https://github.com/SenseRoboticsLab/DISO.git
WORKDIR /home/${USERNAME}/diso_ws
RUN source /opt/ros/lyrical/setup.bash && colcon build --symlink-install

RUN echo "source /opt/ros/lyrical/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /home/${USERNAME}/diso_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash"]
