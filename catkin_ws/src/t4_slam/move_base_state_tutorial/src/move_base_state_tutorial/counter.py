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


class CounterState(EventState):
    """
    ステートの呼び出し回数を記録し，指定回数以上になったら経路を切り替える．
    """

    def __init__(self, target_count=10):
        super(CounterState, self).__init__(outcomes=["over", "pass"])
        self._target_count = target_count
        self._count = 0

    # ==================================================================================================
    #
    #   Flexbe Methods
    #
    # ==================================================================================================
    def on_enter(self, userdata):
        self._count += 1

    def execute(self, userdata: UserData) -> str:
        if self._target_count < self._count:
            return "over"

        return "pass"
