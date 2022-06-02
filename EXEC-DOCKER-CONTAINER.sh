#!/bin/bash

user_name=$1
container_name=$2
command=$3

docker exec -it -u ${user_name} ${container_name} ${command}
