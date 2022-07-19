#!/usr/bin/env python
from __future__ import annotations

import math
from typing import Iterable

import actionlib
import actionlib_msgs.msg as actionlib_msgs
import geometry_msgs.msg as geometry_msgs
import move_base_msgs.msg as move_base_msgs
import rospy
import std_msgs.msg as std_msgs
from tf.transformations import quaternion_from_euler

from flexbe_core import EventState
from flexbe_core.core import UserData


class MoveBaseState(EventState):
    """
    MoveBaseのActionClientのステート
    """

    def __init__(self, action_topic="/move_base", timeout=10):
        super(MoveBaseState, self).__init__(outcomes=["done", "failed"], input_keys=["goal"])
        print(action_topic)
        self._cli = actionlib.SimpleActionClient(action_topic, move_base_msgs.MoveBaseAction)
        self._timeout = rospy.Duration(timeout)

    # ==================================================================================================
    #
    #   Flexbe Methods
    #
    # ==================================================================================================
    def on_start(self) -> None:
        self._cli.wait_for_server()

    def execute(self, userdata: UserData) -> str:
        """
        The userdata.goal format must be [[x,y,z], [rw]].

        """
        target_pose = geometry_msgs.PoseStamped(
            header=std_msgs.Header(stamp=rospy.Time.now(), frame_id="map"),
            pose=self.to_geometry_msgs_pose(userdata.goal)
        )

        action_goal = move_base_msgs.MoveBaseGoal(target_pose)
        self._cli.send_goal(action_goal)
        rospy.loginfo(f"{self.name}: sent goal.")
        self._cli.wait_for_result(timeout=self._timeout)

        if self._cli.get_state() == actionlib_msgs.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"{self.name}: move done.")
            return "done"

        rospy.logwarn(f"{self.name}: move failed.")
        self._cli.cancel_all_goals()
        return "failed"

    # ==================================================================================================
    #
    #   Static Methods (Private)
    #
    # ==================================================================================================
    @classmethod
    def to_geometry_msgs_pose(cls, goal: (geometry_msgs.Pose | Iterable)) -> geometry_msgs.Pose:
        if isinstance(goal, geometry_msgs.Pose):
            return goal

        if isinstance(goal, list):
            position = geometry_msgs.Point(*goal[0])
            radians = math.radians(goal[1][0])
            q = quaternion_from_euler(0., 0., radians)
            orientation = geometry_msgs.Quaternion(q[0], q[1], q[2], q[3])
            return geometry_msgs.Pose(position, orientation)

        raise TypeError(f"Got unexpected type: {type(goal)}")
