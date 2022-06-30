#!/bin/bash

topics="
/head_camera/rgb/image_raw
/tf_static
/tf
"

rosbag record -e ${topics}