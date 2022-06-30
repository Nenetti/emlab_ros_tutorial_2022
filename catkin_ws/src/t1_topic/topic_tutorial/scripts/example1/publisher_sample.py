#! /usr/bin/env python
from pathlib import Path
from typing import Final

import rospy
import std_msgs.msg as std_msgs


class PublisherNode:

    _rate: Final[rospy.Rate]
    _publisher: Final[rospy.Publisher]
    _loop_count: int
    _publish_count: int

    def __init__(self):
        hz = rospy.get_param("~hz")
        self._rate = rospy.Rate(hz)
        self._loop_count = 0
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
        # rospyがshutdownをしていないかのチェック．
        # あくまでこのプログラムのrospyであってroscoreでは無いことに注意
        while not rospy.is_shutdown():

            # Subscriberが存在するかチェック
            # 存在しない場合はPublishしても通信帯域を無駄に専有するだけ．
            if 0 < self._publisher.get_num_connections():
                self._publish_count += 1
                send_text = f"Publish Message No.{self._publish_count}"
                # メッセージをPublish
                self._publisher.publish(std_msgs.String(send_text))
                rospy.loginfo(f"Publish {send_text}")

            self._loop_count += 1
            print(f"Loop: No.{self._loop_count}")
            self._rate.sleep()


if __name__ == '__main__':
    rospy.init_node(Path(__file__).stem)
    node = PublisherNode()
    node.loop()
