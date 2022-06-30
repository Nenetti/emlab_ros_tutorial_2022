#! /usr/bin/env python
from __future__ import annotations

from pathlib import Path
from typing import Any, Final

import rospy
import std_msgs.msg as std_msgs


class SubscriberNode:

    _subscriber_kwargs: Final[dict[str, Any]]
    _rate: Final[rospy.Rate]
    _n_stack: int
    _callback_count: int
    _latest_msgs: list

    def __init__(self):
        self._callback_count = 0
        self._latest_msgs = []

        self._n_stack = 5

        # hzは1秒間の処理回数なので0.5で2秒に1回．
        self._rate = rospy.Rate(hz=0.5)

        self._subscriber_kwargs = dict(
            name="/tutorial_string_topic", data_class=std_msgs.String, callback=self._callback,
            queue_size=None
        )

        # 以下2つは同じ意味
        # dictクラスを直につかった場合，kwargs引数を使って直接keyを指定出来る．(ただしこの場合keyはstr型に必ずなる)
        x = dict(name="/tutorial_string_topic", data_class=std_msgs.String)
        x = {"name": "/tutorial_string_topic", "data_class": std_msgs.String}

    # ------------------------------------------------------------------------------------------------------------------
    #
    #   Public Methods
    #
    # ------------------------------------------------------------------------------------------------------------------
    def loop(self):
        """
        メインループ
        受け取った最新のメッセージ5件を元に処理を行う関数．
        ただし2秒ごとに処理を中断(Sleep)する機能を持つ．
        重要なのはSubscriberのunregisterと再定義．

        """
        while not rospy.is_shutdown():
            sub = rospy.Subscriber(**self._subscriber_kwargs)
            while (len(self._latest_msgs) < self._n_stack) and (not rospy.is_shutdown()):
                # 5件溜まったらwhileから抜け出す．
                rospy.sleep(0.01)

            sub.unregister()
            for msg in self._latest_msgs:
                text = msg.data
                rospy.loginfo(f"    text={text}")

            self._latest_msgs.clear()
            self._rate.sleep()

    # ------------------------------------------------------------------------------------------------------------------
    #
    #   [ROS Subscriber] Callbacks
    #
    # ------------------------------------------------------------------------------------------------------------------
    def _callback(self, msg: std_msgs.String):
        """
        'tutorial_string_topic'のコールバック関数

        Args:
            msg (std_msgs.String):

        """
        self._callback_count += 1

        # msgを追加
        # ただし指定したstack数を超えないように
        if len(self._latest_msgs) < self._n_stack:
            self._latest_msgs.append(msg)
        rospy.loginfo(f"Subscriber No.{self._callback_count}")


if __name__ == '__main__':
    rospy.init_node(Path(__file__).stem)
    node = SubscriberNode()
    rospy.loginfo("Complete to Initialize")

    node.loop()
