# General guidelines and recommendations for writing 'Dockerfile':
# https://docs.docker.com/engine/userguide/eng-image/dockerfile_best-practices/
# ======================================================================================================================
#
#   Initialize Ubuntu Focal Fossa (20.04 LTS) OpenGL support.
#
# ======================================================================================================================
ARG FROM_IMAGE
FROM ${FROM_IMAGE}

# ======================================================================================================================
#
#   Install CUDA 11.2 and cuDNN 8.
#
# ======================================================================================================================
# CUDA 11.3.1 + OpenGL
# https://hub.docker.com/r/nvidia/cudagl/
# https://gitlab.com/nvidia/container-images/cudagl
# https://github.com/NVIDIA/libglvnd

# CUDA 11.3.1-cudnn8-devel-ubuntu20.04
# https://hub.docker.com/r/nvidia/cuda
# https://gitlab.com/nvidia/container-images/cuda
# https://gitlab.com/nvidia/container-images/cuda/-/blob/master/dist/11.3.1/ubuntu2004/devel/cudnn8/Dockerfile

# ------------------------- #
#       Define
# ------------------------- #
ARG cudnn_version=8.2.0.53
ARG nvidia_cudnn_version=${cudnn_version}-1+cuda11.3

# ------------------------- #
#       Install
# ------------------------- #
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn8=${nvidia_cudnn_version} \
    libcudnn8-dev=${nvidia_cudnn_version} \
    && apt-mark hold libcudnn8 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ======================================================================================================================
#
#   Setup Timezone
#
# ======================================================================================================================
# ------------------------- #
#       Define
# ------------------------- #
ARG TIMEZONE="Asia/Tokyo"
ARG TZDATA_VERSION=*

# ------------------------- #
#       Install
# ------------------------- #
# Setup timezone for avoidance of "DEBIAN_FRONTEND=noninteractive".
RUN echo ${TIMEZONE} > /etc/timezone \
    && ln -s /usr/share/zoneinfo/${TIMEZONE} /etc/localtime

# Install tzdata
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata=${TZDATA_VERSION} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ======================================================================================================================
#
#   Install ROS Noetic Ninjemys
#
# ======================================================================================================================
# https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/ros-core/Dockerfile
# https://github.com/osrf/docker_images/blob/master/ros/noetic/debian/buster/ros-base/Dockerfile
# https://github.com/osrf/docker_images/blob/master/ros/noetic/debian/buster/desktop-full/Dockerfile

# ------------------------- #
#       Define
# ------------------------- #
ARG DIRMNGR_VERSION=*
ARG GNUPG2_VERSION=*
ARG WGET_VERSION=*
ARG LSB_RELEASE_VERSION=*

ARG ROS_DESKTOP_FULL_VERSION=*
ARG BUILD_ESSENTIAL_VERSION=*
ARG PYTHON3_ROSDEP_VERSION=*
ARG PYTHON3_ROSINSTALL_VERSION=*
ARG PYTHON3_VCSTOOLS_VERSION=*
ARG PYTHON3_CATKIN_TOOLS_VERSION=*
ARG PYTHON3_OSRF_PYCOMMON_VERSION=*

ARG ROS_FETCH_DESCRIPTION=*


# ------------------------- #
#       Install
# ------------------------- #
# Install dependent packages of installing ROS.
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr=${DIRMNGR_VERSION} \
    gnupg2=${GNUPG2_VERSION} \
    wget=${WGET_VERSION} \
    lsb-release=${LSB_RELEASE_VERSION} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Update the ros package list & Add the package keys.
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -P /tmp \
    && apt-key add /tmp/ros.asc \
    && rm /tmp/ros.asc

# Install the 'ros-noetic-desktop-full' packages (including ROS, Rqt, Rviz, and more).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=${ROS_DESKTOP_FULL_VERSION} \
    build-essential=${BUILD_ESSENTIAL_VERSION} \
    python3-rosdep=${PYTHON3_ROSDEP_VERSION} \
    python3-rosinstall=${PYTHON3_ROSINSTALL_VERSION} \
    python3-vcstools=${PYTHON3_VCSTOOLS_VERSION} \
    python3-catkin-tools=${PYTHON3_CATKIN_TOOLS_VERSION} \
    python3-osrf-pycommon=${PYTHON3_OSRF_PYCOMMON_VERSION} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-fetch-description=${ROS_FETCH_DESCRIPTION} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Initialize 'rosdep'.
# http://wiki.ros.org/rosdep
ENV ROS_DISTRO noetic
RUN rosdep init && rosdep update --rosdistro ${ROS_DISTRO}



# ======================================================================================================================
#
#   Configure audio I/O with ALSA
#
# ======================================================================================================================
# https://www.alsa-project.org/main/index.php/Main_Page

# ------------------------- #
#       Define
# ------------------------- #
ARG LIB_ASOUND2_PLUGINS_VERSION=*

# ------------------------- #
#       Install
# ------------------------- #
RUN apt-get update && apt-get install -y --no-install-recommends \
    libasound2-plugins=${LIB_ASOUND2_PLUGINS_VERSION} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Configure ALSA so that containerized programs can use the audio I/O forwarded to the PulseAudio daemon on the host system.
COPY scripts/asound.conf /etc/asound.conf

# ======================================================================================================================
#
#   Install network packages
#
# ======================================================================================================================
# ------------------------- #
#       Define
# ------------------------- #
ARG INETUTILS_PING_VERSION=*
ARG IPROUTE2_VERSION=*
ARG OPENSSH_CLIENT_VERSION=*

# ------------------------- #
#       Install
# ------------------------- #
RUN apt-get update && apt-get install -y --no-install-recommends \
    inetutils-ping=${INETUTILS_PING_VERSION} \
    iproute2=${IPROUTE2_VERSION} \
    openssh-client=${OPENSSH_CLIENT_VERSION} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ======================================================================================================================
#
#   Install Git
#
# ======================================================================================================================
# ------------------------- #
#       Define
# ------------------------- #
ARG GIT_VERSION=*

# ------------------------- #
#       Install
# ------------------------- #
RUN apt-get update && apt-get install -y --no-install-recommends \
    git=${GIT_VERSION} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ======================================================================================================================
#
#   Install Python3
#
# ======================================================================================================================
# ------------------------- #
#       Define
# ------------------------- #
ARG PYTHON3_DEV_VERSION=3.8.2-0ubuntu2
ARG PYTHON3_PIP_VERSION=20.0.2-5ubuntu1.6

# ------------------------- #
#       Install
# ------------------------- #
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-dev=${PYTHON3_DEV_VERSION} \
    python3-pip=${PYTHON3_PIP_VERSION} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

# ======================================================================================================================
#
#   Install Fetch
#
# ======================================================================================================================
# https://docs.fetchrobotics.com/indigo_to_melodic.html

# ------------------------- #
#       Define
# ------------------------- #
ARG FETCH_ROS_VERSION=*
ARG FETCH_DESCRIPTION=*
ARG ROS_ROBOT_CONTROLLERS=*
ARG ROS_RGBD_LAUNCH=*

# ------------------------- #
#       Install
# ------------------------- #
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-fetch-ros=${FETCH_ROS_VERSION} \
    ros-noetic-fetch-description=${FETCH_DESCRIPTION} \
    ros-noetic-robot-controllers=${ROS_ROBOT_CONTROLLERS} \
    ros-noetic-rgbd-launch=${ROS_RGBD_LAUNCH} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ======================================================================================================================
#
#   User Setup
#
# ======================================================================================================================
# Install sudo package.
# ------------------------- #
#       Define
# ------------------------- #
ARG SUDO_VERSION=*

# ------------------------- #
#       Install
# ------------------------- #
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo=${SUDO_VERSION} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ------------------------- #
#       Setup
# ------------------------- #
ENV DEFAULT_USER=docker
ENV DEFAULT_USER_HOME=/home/${DEFAULT_USER}
ARG DEFAULT_USER_PASSWORD=password
ARG DEFAULT_USER_TEMP_ID=9999
ARG DEFAULT_USER_GROUP_TEMP_ID=65534

RUN useradd -m -s /bin/bash -u ${DEFAULT_USER_TEMP_ID} -g ${DEFAULT_USER_GROUP_TEMP_ID} ${DEFAULT_USER} \
    && echo ${DEFAULT_USER}:${DEFAULT_USER_PASSWORD} | chpasswd

# Set startup script of reconfiguring User ID and Group ID.
COPY scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod 755 /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# ======================================================================================================================
#
#   etc
#
# ======================================================================================================================
# Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /home/${DEFAULT_USER}/.bashrc

# Make catkin_ws directory.
RUN mkdir -p ${DEFAULT_USER_HOME}/catkin_ws/src \
    && chown -R $(id -u ${DEFAULT_USER}):$(id -g ${DEFAULT_USER}) ${DEFAULT_USER_HOME}/catkin_ws

RUN mkdir -p ${DEFAULT_USER_HOME}/test

# Add bash_aliases.
COPY "./scripts/.bash_aliases" "${DEFAULT_USER_HOME}/.bash_aliases"
COPY "./scripts/.bashrc_ros" "${DEFAULT_USER_HOME}/.bashrc_ros"
COPY "./scripts/terminator_roslaunch.sh" "${DEFAULT_USER_HOME}/catkin_ws/terminator_roslaunch.sh"
RUN chown -R $(id -u ${DEFAULT_USER}):$(id -g ${DEFAULT_USER}) \
        "${DEFAULT_USER_HOME}/.bash_aliases" \
        "${DEFAULT_USER_HOME}/.bashrc_ros"  \
        "${DEFAULT_USER_HOME}/catkin_ws/terminator_roslaunch.sh" \
    && chmod 755 \
        "${DEFAULT_USER_HOME}/.bash_aliases" \
        "${DEFAULT_USER_HOME}/.bashrc_ros"  \
        "${DEFAULT_USER_HOME}/catkin_ws/terminator_roslaunch.sh"

# Copy python3 symbolic link.
RUN ln -s /usr/bin/python3 /usr/bin/python

RUN echo "source ${DEFAULT_USER_HOME}/.bashrc_ros" >> ${DEFAULT_USER_HOME}/.bashrc
# Enter the container with a Bash shell.
CMD ["/bin/bash"]