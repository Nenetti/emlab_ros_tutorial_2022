#!/bin/bash

# ----------------------------------------------------------------------------------------------------------------------
#
#   Install Docker
#
# ----------------------------------------------------------------------------------------------------------------------
# https://docs.docker.com/engine/install/ubuntu/

DOCKER_VERSION=*
CONTAINERD_IO_VERSION=*

sudo apt-get remove -y \
  docker \
  docker-engine \
  docker.io \
  containerd \
  runc

sudo apt-get update && sudo apt-get install -y --no-install-recommends \
  ca-certificates \
  curl \
  gnupg \
  lsb-release

sudo rm /usr/share/keyrings/docker-archive-keyring.gpg
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
sudo echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

sudo apt-get update && sudo apt-get install -y --no-install-recommends \
  docker-ce=${DOCKER_VERSION} \
  docker-ce-cli=${DOCKER_VERSION} \
  containerd.io=${CONTAINERD_IO_VERSION}

# ----------------------------------------------------------------------------------------------------------------------
#
#   Install Docker-Compose
#
# ----------------------------------------------------------------------------------------------------------------------
# https://docs.docker.com/compose/install/
# https://github.com/docker/compose/releases
DOCKER_COMPOSE_VERSION="v2.4.1"

sudo curl -L https://github.com/docker/compose/releases/download/"${DOCKER_COMPOSE_VERSION}"/docker-compose-$(uname -s)-$(uname -m) -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# ----------------------------------------------------------------------------------------------------------------------
#
#   Install Nvidia-Docker2
#
# ----------------------------------------------------------------------------------------------------------------------
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker

sudo rm /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
         && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
         && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update && sudo apt-get install -y --no-install-recommends \
  nvidia-docker2

sudo apt-get update && sudo apt-get install -y --no-install-recommends \
  terminator

# Reset default terminal (If you had installed Terminator, the default terminal will be assigned it.)
update-alternatives --set x-terminal-emulator /usr/bin/gnome-terminal.wrapper
# Manual mode
# sudo update-alternatives --config x-terminal-emulator

# ----------------------------------------------------------------------------------------------------------------------
#
#   Git submodules
#
# ----------------------------------------------------------------------------------------------------------------------
git submodule update --init
