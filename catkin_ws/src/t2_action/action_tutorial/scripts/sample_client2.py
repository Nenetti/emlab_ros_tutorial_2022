#! /usr/bin/env python
import time
from pathlib import Path
from typing import Final

import actionlib
import rospy
import std_msgs.msg as std_msgs
import action_tutorial_msgs.msg as action_tutorial_msgs


class AllInOneTemplateNode:

    _action_client: Final[actionlib.SimpleActionClient]

    _sleep_time: Final[int]
    _rate: rospy.Rate
    _is_sleeping: int

    def __init__(self) -> None:
        # -------------------------------------- #
        #   1. Load rosparam & 2. Define Instance Variables
        # -------------------------------------- #
        self._sleep_time = rospy.get_param("~sleep_time")
        self._is_sleeping = False

        # -------------------------------------- #
        #   2 Initialize ROS Action Client
        # -------------------------------------- #
        self._action_client = actionlib.SimpleActionClient("tutorial_action", action_tutorial_msgs.SleepAction)

        # -------------------------------------- #
        #   3. Wait to Start Action Server
        # -------------------------------------- #
        rospy.loginfo(f"Wait Action Server: {self._action_client.action_client.ns}.")
        self._action_client.wait_for_server()
        rospy.loginfo(f"Connect Action Server: {self._action_client.action_client.ns}.")

    # ==================================================================================================================
    #
    #   Public Method
    #
    # ==================================================================================================================
    def start(self) -> None:
        send_count = 0
        while not rospy.is_shutdown():

            # ActionGoalを生成してServerにPublish.
            sleep_time_msg = std_msgs.Float32(self._sleep_time)
            header = std_msgs.Header(seq=send_count, stamp=rospy.Time.now())
            sleep_goal = action_tutorial_msgs.SleepGoal(header, sleep_time_msg)
            self._action_client.send_goal(sleep_goal, feedback_cb=self._action_client_feedback_callback)
            rospy.loginfo(f"Send ActionGoal: No.{header.seq}")

            # Serverからの結果を待ち，成功したかどうかをチェックする.
            is_succeed = self._action_client.wait_for_result()
            if is_succeed:
                result = self._action_client.get_result()
                status_text = self._action_client.get_goal_status_text()
                rospy.loginfo(f"Done ActionServer: result = {result.text.data}")
                rospy.loginfo(f"ActionServer status text: {status_text}")
            else:
                rospy.loginfo(f"Failed ActionResult")
            print()
            send_count += 1

    # ==================================================================================================================
    #
    #   [ROS Action Server] Callbacks
    #
    # ==================================================================================================================
    @staticmethod
    def _action_client_feedback_callback(feedback: action_tutorial_msgs.SleepFeedback) -> None:
        # Server側での処理の途中状況を受け取る.
        rospy.loginfo(f"Feedback: progress = {feedback.progress.data * 100:.0f} %")


if __name__ == '__main__':
    rospy.init_node(Path(__file__).stem)
    node = AllInOneTemplateNode()
    node.start()
