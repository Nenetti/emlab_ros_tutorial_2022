#! /usr/bin/env python
from __future__ import annotations
from pathlib import Path
from typing import Final

import genpy
import rospy
import std_msgs.msg as std_msgs
import rosbag
import sensor_msgs.msg as sensor_msgs
import tf2_msgs.msg as tf2_msgs


class RosbagNode:

    _bag: Final[rosbag.Bag]

    def __init__(self) -> None:
        file_path = rospy.get_param("~file_path")
        self._bag = rosbag.Bag(file_path)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #   Public Methods
    #
    # ------------------------------------------------------------------------------------------------------------------
    def read(self) -> None:
        """
        メインループ
        roscoreが停止するまで指定hzでPublishし続ける．

        """
        for topic, msg, t in self._bag.read_messages():
            topic: str
            msg: genpy.Message
            t: rospy.rostime.Time
            if topic == "/head_camera/rgb/image_raw":
                image = sensor_msgs.Image(*[msg.__getattribute__(key) for key in sensor_msgs.Image.__slots__])

            elif topic == "/tf":
                tf = tf2_msgs.TFMessage(*[msg.__getattribute__(key) for key in tf2_msgs.TFMessage.__slots__])

            elif topic == "/tf_static":
                tf_static = tf2_msgs.TFMessage(*[msg.__getattribute__(key) for key in tf2_msgs.TFMessage.__slots__])


if __name__ == '__main__':
    rospy.init_node(Path(__file__).stem)
    node = RosbagNode()
    node.read()
