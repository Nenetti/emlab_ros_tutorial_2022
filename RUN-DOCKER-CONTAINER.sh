#!/bin/bash

# ==================================================================================================================
#
#   Setup Variable
#
# ==================================================================================================================
export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"
export APP_USER_NAME="app"
export SIMULATOR_USER_NAME="simulator"
#export APP_USER_NAME=${USER}

readonly APP_CONTAINER="emlab-tutorial-client"
readonly SIMULATION_CONTAINER="emlab-tutorial-simulator"
readonly N_SERVICE=2
readonly LOCK_FILE="/tmp/catkin.lock"
readonly COMPLETE_WORD="Complete initialization of entrypoint"
readonly COMPOSE_FILE="./docker/docker-compose.yml"
readonly PIPE=/tmp/entrypoint_monitoring.pipe

# ==================================================================================================================
#
#   [Function] Run container
#
# ==================================================================================================================
function compose_up() {
  # docker-compose up
  docker-compose -f "${COMPOSE_FILE}" up -d
  return $?
}

# ==================================================================================================================
#
#   [Function] Wait for entrypoint
#
# ==================================================================================================================
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

# ==================================================================================================================
#
#   [Function] catkin build
#
# ==================================================================================================================
function catkin_build() {
  docker exec -it -u ${APP_USER_NAME} ${APP_CONTAINER} /bin/bash -i -c "source /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin build"
  docker exec -it -u ${SIMULATOR_USER_NAME} ${SIMULATION_CONTAINER} /bin/bash -i -c "source /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin build"
}

# ==================================================================================================================
#
#   main
#
# ==================================================================================================================
cd $(dirname "$0")
if [ "$(docker inspect --format="{{.State.Status}}" ${APP_CONTAINER})" != "running" ]; then
    # Generate lock file
  if [ ! -e ${LOCK_FILE} ]; then
    touch ${LOCK_FILE}
  fi

  compose_up
  if [ $? != 0 ]; then
    rm ${LOCK_FILE}
    exit 1
  fi

  wait_for_entrypoint
  catkin_build

  rm ${LOCK_FILE}

  exit 0
fi

if [ ! -e ${LOCK_FILE} ]; then
  echo "Containers are already activated."
  exit 0
fi

echo "Wait for docker-compose up ..."
while [ -e ${LOCK_FILE} ]; do
  sleep 0.1
done
