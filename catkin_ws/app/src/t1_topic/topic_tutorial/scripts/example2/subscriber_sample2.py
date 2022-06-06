#! /usr/bin/env python
from pathlib import Path
from typing import Final

import rospy
import std_msgs.msg as std_msgs


class SubscriberNode:

    _subscribe_topic: Final[str]
    _rate: Final[rospy.Rate]
    _subscribe_topic_type: type
    _subscribe_count: int

    def __init__(self):
        self._subscribe_count = 0

        # hzは1秒間の処理回数なので0.5で2秒に1回．
        self._rate = rospy.Rate(hz=1)

        self._subscribe_topic = "/tutorial_string_topic"
        self._subscribe_topic_type = std_msgs.String

    # ------------------------------------------------------------------------------------------------------------------
    #
    #   Public Methods
    #
    # ------------------------------------------------------------------------------------------------------------------
    def loop(self):
        """
        メインループ
        受け取った最新のメッセージを元に処理を行う関数．
        ただし受け取るたびに2秒，処理を中断(Sleep)する機能を持つ．

        """
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message(self._subscribe_topic, self._subscribe_topic_type)

            self._subscribe_count += 1
            # CompressedImageのROSメッセ−ジをnumpyに変換．
            text = msg.data
            rospy.loginfo(f"Subscriber No.{self._subscribe_count}: seq={text}")
            self._rate.sleep()


if __name__ == '__main__':
    rospy.init_node(Path(__file__).stem)
    node = SubscriberNode()
    rospy.loginfo("Complete to Initialize")

    node.loop()
