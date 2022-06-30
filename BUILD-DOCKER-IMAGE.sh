#!/bin/bash

docker-compose -f ./docker/docker-compose-build.yml stop
docker-compose -f ./docker/docker-compose-build.yml rm -f
docker-compose -f ./docker/docker-compose-build.yml build
