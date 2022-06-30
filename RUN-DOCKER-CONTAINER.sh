#!/bin/bash

# ----------------------------------------------------------------------------------------------------------------------
#
#   Setup Variable
#
# ----------------------------------------------------------------------------------------------------------------------
cd $(dirname "$0")

export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

compose_file_relative_path="./docker/docker-compose.yml"
override_gpu_file_relative_path="./docker/docker-compose-gpu.override.yml"

export COMPOSE_FILE=$(cd $(dirname "${compose_file_relative_path}") && pwd)/$(basename "${compose_file_relative_path}")
override_gpu_file=$(cd $(dirname "${override_gpu_file_relative_path}") && pwd)/$(basename "${override_gpu_file_relative_path}")

if [ -e /proc/driver/nvidia/version ]; then
  COMPOSE_FILE="${COMPOSE_FILE}:${override_gpu_file}"
fi

container_name="emlab-tutorial-client"

readonly LOCK_FILE="/tmp/catkin.lock"
readonly PIPE=/tmp/entrypoint_monitoring.pipe

# initial completed word of entrypoint
readonly COMPLETE_WORD="Complete initialization of entrypoint"

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

  docker-compose logs -f --since 0s | tee ${PIPE} >/dev/null &
  local PID=$!

  local count=0
  local line
  cat ${PIPE} | while read line; do
    echo ${line}
    if [[ ${line} =~ ${COMPLETE_WORD} ]]; then
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
function catkin_build_in_container() {
  local user_name=$1
  local container_name=$2
  docker exec -it -u ${user_name} ${container_name} /bin/bash -i -c "cd ~/catkin_ws; catkin build"
  if [ $? != 0 ]; then
    exit_command "Failed to catkin build in ${container_name}"
  fi
}

# ----------------------------------------------------------------------------------------------------------------------
#
#   [Function] exit process
#
# ----------------------------------------------------------------------------------------------------------------------
function exit_command() {
  local error_text=$1
  docker-compose stop
  if [ -e ${LOCK_FILE} ]; then
    rm ${LOCK_FILE}
  fi
  echo "${error_text}"
  exit 1
}

# ----------------------------------------------------------------------------------------------------------------------
#
#   Main
#
# ----------------------------------------------------------------------------------------------------------------------
# If container does not up.
if [ "$(docker inspect --format="{{.State.Status}}" ${container_name})" != "running" ]; then
  trap "exit_command 'CTRL+C Interrupted' " SIGINT

  # Generate lock file
  if [ ! -e ${LOCK_FILE} ]; then
    touch ${LOCK_FILE}
  fi

  # docker-compose up -d
  docker-compose up -d

  if [ $? != 0 ]; then
    exit_command "Failed docker-compose up"
  fi

  # Wait to complete the entrypoint with all containers.
  wait_for_entrypoint

  catkin_build_in_container docker ${container_name}

  rm ${LOCK_FILE}

elif [ ! -e ${LOCK_FILE} ]; then
  # if container already upped.
  echo "Containers are already activated."

else
  # running entrypoint.
  echo "Wait for docker-compose up ..."
  while [ -e ${LOCK_FILE} ]; do
    sleep 0.1
  done
fi
