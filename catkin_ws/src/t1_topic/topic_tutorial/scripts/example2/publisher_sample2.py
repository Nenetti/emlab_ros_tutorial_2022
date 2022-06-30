#! /usr/bin/env python
from pathlib import Path
from typing import Final

import numpy as np
import rospy
import std_msgs.msg as std_msgs


class PublisherNode:

    _rate: Final[rospy.Rate]
    _publisher: Final[rospy.Publisher]
    _publish_count: int

    def __init__(self):
        self._rate = rospy.Rate(hz=20)
        self._publish_count = 0
        self._publisher = rospy.Publisher("/tutorial_string_topic", std_msgs.String, queue_size=1)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #   Public Methods
    #
    # ------------------------------------------------------------------------------------------------------------------
    def loop(self):
        """
        メインループ
        roscoreが停止するまで指定hzでPublishし続ける．

        """
        while not rospy.is_shutdown():
            self._publish_count += 1

            send_text = f"Publish Message No.{self._publish_count}"
            self._publisher.publish(std_msgs.String(send_text))

            rospy.loginfo(f"Publish {self._publish_count}")
            self._rate.sleep()


if __name__ == '__main__':
    rospy.init_node(Path(__file__).stem)
    node = PublisherNode()
    rospy.loginfo("Complete to Initialize")

    node.loop()
