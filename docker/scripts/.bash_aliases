#!/bin/bash
# ======================================================================================================================
#
#   Overwrite catkin command
#
# ======================================================================================================================
function catkin() {
  SOURCE_DIR=${HOME}/catkin_ws/devel/.private
  TARGET_DIR=${HOME}/catkin_ws/dist-packages
  SETUP_BASH_FILE=${HOME}/catkin_ws/devel/setup.bash
  current_directory=$(pwd)

  # Execute catkin command
  /usr/bin/catkin $@

  status=$?
  if [ ${status} == 0 ]; then
    # Copy dist-packages
    cd ${SOURCE_DIR}
    rm -R ${TARGET_DIR}/*
    for item in `find -name "dist-packages" -type d` ; do
      path=${item}/$(echo "${item}" | cut -d "/" -f 2)
      cp -r ${path} ${TARGET_DIR}
    done
    cd ${current_directory}

    # Load setup.bash
    if [ -e ${SETUP_BASH_FILE} ]; then
      source ${SETUP_BASH_FILE}
      echo ${SETUP_BASH_FILE}
    fi
  fi
}
