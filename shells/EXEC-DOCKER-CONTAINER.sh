#!/bin/bash

container_name=$1
command=$2

docker exec -it -u docker ${container_name} ${command}

bash