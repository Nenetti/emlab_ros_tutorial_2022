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
        r = rospy.Rate(100)
        send_count = 0
        while not rospy.is_shutdown():
            if self._is_sleeping:
                r.sleep()
                continue

            # ActionGoalを生成してServerにPublish.
            sleep_time_msg = std_msgs.Float32(self._sleep_time)
            header = std_msgs.Header(seq=send_count, stamp=rospy.Time.now())
            sleep_goal = action_tutorial_msgs.SleepGoal(header, sleep_time_msg)
            self._action_client.send_goal(
                sleep_goal, done_cb=self._action_client_done_callback, feedback_cb=self._action_client_feedback_callback
            )
            rospy.loginfo(f"Send ActionGoal: No.{header.seq}")
            send_count += 1
            self._is_sleeping = True
            r.sleep()

    # ==================================================================================================================
    #
    #   [ROS Action Server] Callbacks
    #
    # ==================================================================================================================
    def _action_client_done_callback(
            self, status: actionlib.SimpleGoalState, result: action_tutorial_msgs.SleepResult
    ) -> None:
        # Serverからの結果を待ち，成功したかどうかをチェックする.
        self._is_sleeping = False
        rospy.loginfo(
            f"Done Action Server: status={status}, result = {result.text.data}"
        )
        print()

    @staticmethod
    def _action_client_feedback_callback(feedback: action_tutorial_msgs.SleepFeedback) -> None:
        # Server側での処理の途中状況を受け取る.
        rospy.loginfo(f"Feedback: progress = {feedback.progress.data * 100:.0f} %")


if __name__ == '__main__':
    rospy.init_node(Path(__file__).stem)
    node = AllInOneTemplateNode()
    node.start()
