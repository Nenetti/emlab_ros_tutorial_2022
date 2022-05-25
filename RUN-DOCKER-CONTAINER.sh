#!/bin/bash

# ==================================================================================================================
#
#   Setup Variable
#
# ==================================================================================================================
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)
export APP_USER_NAME=noetic
export SIMULATOR_USER_NAME=simulator
#export APP_USER_NAME=${USER}

APP_SERVICE=emlab-tutorial-client
SIMULATION_SERVICE=emlab-tutorial-simulator
N_SERVICE=2

if [ -e /proc/driver/nvidia/version ]; then
  export DOCKER_RUNTIME=nvidia
else
  export DOCKER_RUNTIME=runc
fi

# ==================================================================================================================
#
#   Run container
#
# ==================================================================================================================
LOCK_FILE=/tmp/catkin.lock
if [ "$(docker inspect --format="{{.State.Status}}" ${APP_SERVICE})" != "running" ]; then
  # Generate lock file
  if [ ! -e ${LOCK_FILE} ]; then
    touch ${LOCK_FILE}
  fi

  # docker-compose up
  cd $(dirname "$0")
  docker-compose -f ./docker/docker-compose.yml up -d
  compose_status=$?
  if [ ${compose_status} != 0 ]; then
    rm ${LOCK_FILE}
    exit 1
  fi

  # ==================================================================================================================
  #
  #   Wait for entrypoint
  #
  # ==================================================================================================================
  CompleteWord="Complete initialization of entrypoint"

  PIPE=/tmp/entrypoint_monitoring.pipe
  if [ -e ${PIPE} ]; then
    rm ${PIPE}
  fi
  mkfifo ${PIPE}
  docker-compose -f ./docker/docker-compose.yml logs -f --since 0s | tee ${PIPE} >/dev/null &
  PID=$!
  count=0

  cat ${PIPE} | while read line; do
    echo ${line}
    if [[ ${line} =~ ${CompleteWord} ]]; then
      count=$((count+1))
    fi
    if [[ ${count} == ${N_SERVICE} ]]; then
      kill $PID
      break
    fi
  done

  rm ${PIPE}

  docker exec -it -u ${APP_USER_NAME} ${APP_SERVICE} /bin/bash -i -c "source /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin build"
  docker exec -it -u ${SIMULATOR_USER_NAME} ${SIMULATION_SERVICE} /bin/bash -i -c "source /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin build"
  rm ${LOCK_FILE}

else
  # ==================================================================================================================
  #
  #   Wait docker-compose up
  #
  # ==================================================================================================================
  if [ -e ${LOCK_FILE} ]; then
    echo "Wait for docker-compose up ..."
  fi

  while [ -e ${LOCK_FILE} ]; do
    sleep 0.1
  done

fi

# ==================================================================================================================
#
#   Enter the docker container
#
# ==================================================================================================================
#docker exec -it -u ${APP_USER_NAME} ${APP_SERVICE} /bin/bash
