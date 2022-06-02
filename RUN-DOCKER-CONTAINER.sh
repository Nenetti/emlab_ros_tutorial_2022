#!/bin/bash

# ----------------------------------------------------------------------------------------------------------------------
#
#   Setup Variable
#
# ----------------------------------------------------------------------------------------------------------------------
export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"
export APP_USER_NAME="app"
export SIMULATOR_USER_NAME="simulator"
#export APP_USER_NAME=${USER}

readonly APP_CONTAINER="emlab-tutorial-client"
readonly SIMULATION_CONTAINER="emlab-tutorial-simulator"
readonly N_SERVICE=1
readonly LOCK_FILE="/tmp/catkin.lock"
readonly COMPOSE_FILE="./docker/docker-compose-simulator.yml"
readonly PIPE=/tmp/entrypoint_monitoring.pipe

# initial completed word of entrypoint
readonly COMPLETE_WORD="Complete initialization of entrypoint"

# ----------------------------------------------------------------------------------------------------------------------
#
#   [Function] Run container
#
# ----------------------------------------------------------------------------------------------------------------------
function docker_compose_up() {
  # docker-compose up
  docker-compose -f "${COMPOSE_FILE}" up -d
  return $?
}

# ----------------------------------------------------------------------------------------------------------------------
#
#   [Function] Wait for entrypoint
#
# ----------------------------------------------------------------------------------------------------------------------
function wait_for_entrypoint() {
  if [ -e ${PIPE} ]; then
    rm ${PIPE}
  fi
  mkfifo ${PIPE}

  docker-compose -f "${COMPOSE_FILE}" logs -f --since 0s | tee ${PIPE} >/dev/null &
  PID=$!

  local count=0
  local line
  cat ${PIPE} | while read line; do
    echo ${line}
    if [[ ${line} =~ ${COMPLETE_WORD} ]]; then
      count=$((count + 1))
    fi
    if [[ ${count} == ${N_SERVICE} ]]; then
      kill $PID
      break
    fi
  done

  rm ${PIPE}
}

# ----------------------------------------------------------------------------------------------------------------------
#
#   [Function] catkin build with docker run
#
# ----------------------------------------------------------------------------------------------------------------------
function catkin_build() {
  docker exec -it -u ${APP_USER_NAME} ${APP_CONTAINER} /bin/bash -i -c "source /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin build"
  docker exec -it -u ${SIMULATOR_USER_NAME} ${SIMULATION_CONTAINER} /bin/bash -i -c "source /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin build"
}

# ----------------------------------------------------------------------------------------------------------------------
#
#   Main
#
# ----------------------------------------------------------------------------------------------------------------------
cd $(dirname "$0")

# If container does not up.
if [ "$(docker inspect --format="{{.State.Status}}" ${APP_CONTAINER})" != "running" ]; then
  # Generate lock file
  if [ ! -e ${LOCK_FILE} ]; then
    touch ${LOCK_FILE}
  fi

  # docker-compose up -d
  docker_compose_up
  if [ $? != 0 ]; then
    # Failed to run container
    rm ${LOCK_FILE}
    exit 1
  fi

  # Wait to complete the entrypoint with all containers.
  wait_for_entrypoint
  catkin_build

  rm ${LOCK_FILE}
  exit 0
fi

# if container already upped.
if [ ! -e ${LOCK_FILE} ]; then
  echo "Containers are already activated."
  exit 0
fi

# running entrypoint.
echo "Wait for docker-compose up ..."
while [ -e ${LOCK_FILE} ]; do
  sleep 0.1
done
