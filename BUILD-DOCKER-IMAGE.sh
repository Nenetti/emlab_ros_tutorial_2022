#!/bin/bash

SERVICE=noetic-tutorial

docker stop ${SERVICE}
docker rm ${SERVICE}

docker-compose -f ./docker/docker-compose.yml build
