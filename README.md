# emlab_ros_tutorial_2022
Emergent Systems Laboratory ROS1 Tutorial 2022
<!--
# ----------------------------------------------------------------------------------------------------------------------
#
#   Overview
#
# ----------------------------------------------------------------------------------------------------------------------
--->

# Overview <a id="Overview"></a>

**Content:**

* [Overview](#Overview)
* [Getting Started](#Setup)

<!--
# ----------------------------------------------------------------------------------------------------------------------
#
#   Setup
#
# ----------------------------------------------------------------------------------------------------------------------
--->
# Getting Started <a id="Setup"></a>

Getting Started: [docs/getting_started.md](docs/getting_started.md).

ビルドの要約
(詳細は上記 `docs/getting_started.md`を参照)
```
sudo apt-get update && sudo apt-get install -y git 
git clone https://github.com/Nenetti/emlab_ros_tutorial_2022.git
cd emlab_ros_tutorial_2022
bash ./SETUP-DEVELOPMENT-ENVIRONMENT.sh
bash ./BUILD-DOCKER-IMAGE.sh
```

Docker起動コマンド要約
```
bash ./RUN-DOCKER-CONTAINER.sh
docker exec -it -u app emlab-tutorial-client bash
```

<!--
# ----------------------------------------------------------------------------------------------------------------------
#
#   Example
#
# ----------------------------------------------------------------------------------------------------------------------
--->

## Example <a id="Example"></a>

Start example.

    roslaunch rviz_gaussian_distribution example.launch

Start the server.

    roslaunch rviz_gaussian_distribution server.launch
