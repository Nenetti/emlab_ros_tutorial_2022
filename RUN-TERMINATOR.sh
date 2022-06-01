#!/bin/bash

cd $(dirname "$0")
bash RUN-DOCKER-CONTAINER.sh
terminator -g ./terminator/config -l emlab