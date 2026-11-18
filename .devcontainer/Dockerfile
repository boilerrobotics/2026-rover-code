
# Stage 1: ROS 2 Humble + Python + C++ + PCL + RViz2 + Gazebo

FROM osrf/ros:humble-desktop

ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# ----------------------------------------------------------------------
# System tools, Python, GCC/G++, CMake, and PCL
# ----------------------------------------------------------------------

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Python development
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    \
    # General development tools
    git \
    curl \
    wget \
    nano \
    vim \
    bash-completion \
    sudo \
    \
    # C++ development
    build-essential \
    gcc \
    g++ \
    cmake \
    pkg-config \
    \
    # Point Cloud Library
    libpcl-dev \
    pcl-tools \
    libeigen3-dev \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    \
    # GUI / graphics support
    mesa-utils \
    libgl1-mesa-dri \
    x11-apps \
    \
    # RQT plugins
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-tf-tree \
    ros-humble-rqt-robot-steering \
    \
    # ROS bags + MCAP
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    \
    # Gazebo Fortress + ROS 2 integration
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim-demos \
    \
    # ROS 2 control
    ros-humble-gz-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir gdown mcap

# ----------------------------------------------------------------------
# GUI / graphics environment
# ----------------------------------------------------------------------

ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_DRIVER_CAPABILITIES=all

# ----------------------------------------------------------------------
# Python scientific libraries
# ----------------------------------------------------------------------

RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib

# ----------------------------------------------------------------------
# Create non-root development user
# ----------------------------------------------------------------------

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ----------------------------------------------------------------------
# Automatically source ROS 2 Humble in every terminal
# ----------------------------------------------------------------------

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# ----------------------------------------------------------------------
# ROS dependencies
# ----------------------------------------------------------------------

USER $USERNAME

RUN rosdep update

# ----------------------------------------------------------------------
# Default shell
# ----------------------------------------------------------------------

CMD ["/bin/bash"]
