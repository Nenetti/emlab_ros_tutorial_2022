#!/bin/bash

container_user_name="${1}"
container_name="${2}"
package_name="${3}"
launch_file_name=${4}

terminator_command="${package_name} ${launch_file_name}"
command="export \"TERMINATOR_LAUNCH=${terminator_command}\" && bash -i"
echo "${command}"

docker exec -it -u ${container_user_name} ${container_name} bash -c "${command}"
