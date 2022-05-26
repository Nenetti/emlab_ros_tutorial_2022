#!/bin/bash

cd $(dirname "$0")
./RUN-DOCKER-CONTAINER.sh
terminator -g ./terminator/config -l emlab