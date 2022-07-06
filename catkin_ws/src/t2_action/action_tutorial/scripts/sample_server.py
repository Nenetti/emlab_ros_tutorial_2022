#! /usr/bin/env python
from pathlib import Path
from typing import Final

import action_tutorial_msgs.msg as action_tutorial_msgs
import actionlib
import rospy
import std_msgs.msg as std_msgs


class TemplateServerNode:

    _action_server: Final[actionlib.SimpleActionServer]

    def __init__(self) -> None:
        # -------------------------------------- #
        #   1. Initialize ROS Action Server
        # -------------------------------------- #
        # auto_startはデフォルト値ではTrueになっているが必ずFalseにすること
        # 本プログラムの様にActionServerを定義してから開始するまでに何の処理も無い時もFalse
        # なぜならauto_startをTrueにすると常にFalseにしろとWarningが出るから．
        self._action_server = actionlib.SimpleActionServer(
            "tutorial_action", action_tutorial_msgs.SleepAction, self._action_callback, auto_start=False
        )
        # -------------------------------------- #
        #   2. Start ROS Server
        # -------------------------------------- #
        # ActionServerは定義するだけでは開始しない．start()を呼び出して初めて開始される．
        self._action_server.start()
        rospy.loginfo(f"Ready to ActionServer: namespace={self._action_server.action_server.ns}")

    # ==================================================================================================================
    #
    #   [ROS Action Server] Callbacks
    #
    # ==================================================================================================================
    def _action_callback(self, goal: action_tutorial_msgs.SleepGoal) -> None:
        rospy.loginfo(f"Callback ActionGoal: No.{goal.header.seq}")

        sleep_time = rospy.Duration(goal.time.data)
        start_time = rospy.Time.now()
        rate = rospy.Rate(hz=10)

        rospy.loginfo("Wait for completing the request process.")
        while not rospy.is_shutdown():
            elapsed_time = rospy.Time.now() - start_time

            # Clientから指定されたSleep時間を超えた場合はループを抜ける．
            if sleep_time < elapsed_time:
                break

            # ActionServerからClientに向けたFeedback(処理の途中状況)を送信．
            progress_msg = std_msgs.Float32((elapsed_time.to_sec() / sleep_time.to_sec()))
            feedback = action_tutorial_msgs.SleepFeedback(progress_msg)
            self._action_server.publish_feedback(feedback)

            rate.sleep()

        # Clientに処理の終了を報告．
        result_msg = action_tutorial_msgs.SleepResult(text=std_msgs.String("DONE!"))
        self._action_server.set_succeeded(result_msg)
        rospy.loginfo("Done Action.")
        rospy.loginfo("Send ActionResult.")
        print()


if __name__ == '__main__':
    rospy.init_node(Path(__file__).stem)
    node = TemplateServerNode()
    rospy.spin()
