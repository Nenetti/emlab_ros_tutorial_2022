#!/bin/bash
################################################################################
function kill_node( ){
  local pid="$1"
  if [ -n "${pid}" ]; then
    child_pids="$(ps -ef | grep "${pid}" | grep -v grep | awk '{ print $2 }')"
    child_pids="$(echo ${child_pids} | sed -e "s/[\r\n]\+//g")"
#    echo "${pid}, $child_pids"
    for p in ${child_pids}; do
      if [ "${pid}" != "${p}" ]; then
        kill_node "${p}"
#        echo KILL "${p}"
      fi
    done
    kill -9 "${pid}"
  fi
}

function on_signal_interrupt( ){
  if [ -n "${launch_pid}" ]; then
    kill_node "${launch_pid}"
  fi
}

# CTRL+C signal
trap on_signal_interrupt 2

PKG="${1}"
LAUNCH="${2}"

PROMPT_START=$'\e[4m\e[1m'
PROMPT_END=$'\e[0m'
PROMPT="${PROMPT_START}
Run '${PKG} ${LAUNCH}'?
Press: Enter${PROMPT_END}
"

while true; do
  read -n 1 -s -p "${PROMPT}" input
  if [ "${input}" = "" ]; then
#    echo "ROS_MASTER_URI: ${ROS_MASTER_URI}"

    roslaunch ${PKG} ${LAUNCH} &
    launch_pid=$!
    process_cmd="/usr/bin/python3 /opt/ros/noetic/bin/roslaunch ${PKG} ${LAUNCH}"

    while true; do
      if [ -z "$(ps aux | grep "${launch_pid}" | grep -v grep)" ]; then
        break
      fi
      sleep 1.0
    done
    unset launch_pid
    unset process_cmd
    clear
  fi
done