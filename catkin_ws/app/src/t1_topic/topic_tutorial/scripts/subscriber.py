#! /usr/bin/env python
# 1行目の#!はこのファイルを以降のコマンドで実行するという意味
# これによって ./ (execコマンド) で実行された際にファイルから実行ソフト(bash, python, python3等)を指定出来る．
# /usr/bin/env は環境変数を引き継いだ状態で後ろのコマンドを実行という意味
# よって "#! /usr/bin/env python" は，terminalで "python subscriber.py" を叩くのと同じ
import rospy
import std_msgs.msg as std_msgs


class SubscriberNode:

    def __init__(self):
        self._callback_count = 0
        self._subscriber = rospy.Subscriber("/tutorial_string_topic", std_msgs.String, callback=self._callback)

    # ==================================================================================================================
    #
    #   [ROS Subscriber] Callbacks
    #
    # ==================================================================================================================
    def _callback(self, msg: std_msgs.String):
        """
        'tutorial_string_topic'のコールバック関数

        Args:
            msg (std_msgs.String):

        """
        text = msg.data
        self._callback_count += 1
        rospy.loginfo(f"Subscriber No.{self._callback_count}, Text: {text}")


if __name__ == '__main__':
    rospy.init_node("a")
    node = SubscriberNode()
    rospy.spin()
