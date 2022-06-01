#!/bin/bash

readonly CATKIN_WS_SETUP_BASH_FILE="${HOME}/catkin_ws/devel/setup.bash"
readonly OPT_SETUP_BASH_FILE="/opt/ros/noetic/setup.bash"

function catkin() {
  local -r SOURCE_DIR="${HOME}/catkin_ws/devel/.private"
  local -r TARGET_DIR="${HOME}/catkin_ws/dist-packages"
  local -r current_directory=$(pwd)

  # Execute catkin command
  /usr/bin/catkin $@
  local -r status=$?
  if [ ${status} != 0 ]; then
    return ${status}
  fi

  # Load setup.bash
  if [ -e "${CATKIN_WS_SETUP_BASH_FILE}" ]; then
    echo "source ${CATKIN_WS_SETUP_BASH_FILE}"
    source "${CATKIN_WS_SETUP_BASH_FILE}"
  fi

  # When catkin successfully (build, clean, etc...)
  # Copy dist-packages
  if [ -n "$(ls ${TARGET_DIR})" ]; then
    # Clear directory
    rm -R ${TARGET_DIR}/*
  fi

  cd ${SOURCE_DIR}
  for item in $(find -name "dist-packages" -type d); do
    path=${item}/$(echo "${item}" | cut -d "/" -f 2)
    cp -r ${path} ${TARGET_DIR}
  done
  cd ${current_directory}
}

function source_catkin_setup_bash() {
  if [ -e ${CATKIN_WS_SETUP_BASH_FILE} ]; then
    echo "source ${CATKIN_WS_SETUP_BASH_FILE}"
    source ${CATKIN_WS_SETUP_BASH_FILE}
  else
    echo "source ${OPT_SETUP_BASH_FILE}"
    source /opt/ros/noetic/setup.bash
  fi
}

# Load ROS
source_catkin_setup_bash

# Forbid to generate __pycache__
PYTHONDONTWRITEBYTECODE=1

# For Rviz
export XDG_RUNTIME_DIR=/run/user/$(id -u)
