#!/bin/bash

SERVICE=ros-noetic
docker stop ${SERVICE}
docker rm ${SERVICE}