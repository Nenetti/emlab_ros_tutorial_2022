#! /usr/bin/env python


from pathlib import Path

import rospy
import std_msgs.msg as std_msgs


# example1/subscriber_sample.py のだめな書き方．
# 動くけどダメ

class BADSubscriberNode:

    # 型定義が書かれていない．

    def __init__(self):
        self._subscriber = rospy.Subscriber("/tutorial_string_topic", std_msgs.String, callback=self._callback)

        # ...

        # Finalで定義して無い場合に同じ変数に2重に定義出来てしまう．
        # Finalを定義しておけばIDEなら警告出してくれる．
        self._subscriber = rospy.Subscriber("/tutorial_string_topic", std_msgs.String, callback=self._callback)

        # Subscriberの下に変数が定義されている．
        # プログラムの起動順によってエラーが出たり出なかったり．
        # 挙動が安定しない最悪な書き方．
        self._callback_count = 0

    # msgに関する型の説明がないせいで，型がSubscriberの定義部分まで遡らないといけない．
    def _callback(self, msg):
        text = msg.data

        # __init__内で_callback_countをSubscriberの下に定義をしていると，ここで中確率で undefined のエラーが出る．
        self._callback_count += 1
        rospy.loginfo(f"Subscriber No.{self._callback_count}, Text: {text}")


if __name__ == '__main__':
    rospy.init_node(Path(__file__).stem)
    node = BADSubscriberNode()
    rospy.spin()
