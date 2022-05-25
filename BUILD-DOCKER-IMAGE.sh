#!/bin/bash

docker-compose -f ./docker/docker-compose.yml stop
docker-compose -f ./docker/docker-compose.yml rm -f
docker-compose -f ./docker/docker-compose.yml build
