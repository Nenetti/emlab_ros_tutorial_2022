#!/bin/bash

# ----------------------------------------------------------------------------------------------------------------------
#
#   [Function] source setup.bash
#
# ----------------------------------------------------------------------------------------------------------------------
function source_catkin_setup_bash() {
  local -r CATKIN_WS_SETUP_BASH_FILE="${HOME}/catkin_ws/devel/setup.bash"
  local -r OPT_SETUP_BASH_FILE="/opt/ros/noetic/setup.bash"

  if [ -e ${CATKIN_WS_SETUP_BASH_FILE} ]; then
    echo "source ${CATKIN_WS_SETUP_BASH_FILE}"
    source ${CATKIN_WS_SETUP_BASH_FILE}
  else
    echo "source ${OPT_SETUP_BASH_FILE}"
    source /opt/ros/noetic/setup.bash
  fi
}

# ----------------------------------------------------------------------------------------------------------------------
#
#   [Function] ROS environment
#
# ----------------------------------------------------------------------------------------------------------------------
function setup_ros_environment() {
  echo

  # ROS_IP
  if [ -n "${ROS_HOSTNAME}" ] && [ -n "${ROS_IP}" ]; then
    echo "ROS_IP (${ROS_IP}) was deleted. ROS_HOSTNAME is always prioritized."
    unset ROS_IP
  elif [ -z "${ROS_HOSTNAME}" ] && [ -z "${ROS_IP}" ]; then
    ROS_IP=$(hostname -I | cut -d " " -f 1)
    echo "ROS_IP was set automatically. ROS_IP or ROS_HOSTNAME must be set."
  fi

  echo "  ROS_IP: ${ROS_IP}"
  echo "  ROS_HOSTNAME: ${ROS_HOSTNAME}"
  echo "  ROS_MASTER_URI: ${ROS_MASTER_URI}"
  echo
}

# ----------------------------------------------------------------------------------------------------------------------
#
#   Main
#
# ----------------------------------------------------------------------------------------------------------------------
# Load ROS
source_catkin_setup_bash
setup_ros_environment

# For Rviz
export XDG_RUNTIME_DIR=/run/user/$(id -u)
