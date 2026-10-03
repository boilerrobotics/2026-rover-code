# Stage 1 dev environment: ROS2 Humble + rclpy + rviz2 + colcon
# Uses the "desktop" flavor (not ros-base) specifically because it bundles
# rviz2, rqt, and other GUI tools that ros-base leaves out.
FROM osrf/ros:humble-desktop

ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# --- Base tooling -------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    git curl wget nano vim bash-completion sudo \
    # GUI / graphics support
    mesa-utils \
    libgl1-mesa-dri \
    x11-apps \
    # extra rqt plugins (many are already in desktop, these are harmless)
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-tf-tree \
    ros-humble-rqt-robot-steering \
    && rm -rf /var/lib/apt/lists/*
    && pip3 install --no-cache-dir gdown

# Helps Qt apps (rqt, rviz2) behave over X11 forwarding
ENV QT_X11_NO_MITSHM=1
# Only matters if you have an NVIDIA GPU + nvidia-container-toolkit
ENV NVIDIA_DRIVER_CAPABILITIES=all

# numpy + a few Python libs that come up constantly in ROS2/robotics code.
# rclpy itself is already provided by the base ROS2 image.
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib

# --- rosdep ---------------------------------------------------------------

# --- Non-root user (VS Code / devcontainers best practice) -----------------
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Source the base ROS2 environment automatically in every new shell.
# The workspace's own install/setup.bash is sourced manually after
# `colcon build` (see README) since its path depends on where VS Code
# mounts the repo, which varies by repo folder name.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME

RUN rosdep update

CMD ["/bin/bash"]
