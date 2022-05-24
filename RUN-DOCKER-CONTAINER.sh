#!/bin/bash

# ==================================================================================================================
#
#   Setup Variable
#
# ==================================================================================================================
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)
export USER_NAME=noetic
#export USER_NAME=${USER}

SERVICE=noetic-tutorial

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
if [ "$(docker inspect --format="{{.State.Status}}" ${SERVICE})" != "running" ]; then
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

  cat ${PIPE} | while read line; do
    echo ${line}
    if [[ ${line} =~ ${CompleteWord} ]]; then
      kill $PID
      break
    fi
  done

  rm ${PIPE}

  docker exec -it -u ${USER_NAME} noetic-tutorial /bin/bash -i -c "source /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin build"
  rm ${LOCK_FILE}

else

  if [ -e ${LOCK_FILE} ]; then
    echo "Wait for docker-compose up ..."
  fi

  while [ -e ${LOCK_FILE} ]
  do
    sleep 0.1
  done

fi

# ==================================================================================================================
#
#   Enter the docker container
#
# ==================================================================================================================
docker exec -it -u ${USER_NAME} noetic-tutorial /bin/bash
