#!/usr/bin/env python

import rospy

from flexbe_core import EventState
from flexbe_core.core import UserData


class PrintState(EventState):
    """
    入力をrospy.loginfoで出力するだけ．
    """

    def __init__(self) -> None:
        super(PrintState, self).__init__(outcomes=["done"], input_keys=["print_data"])

    def execute(self, userdata: UserData):
        rospy.loginfo(f"print: f{userdata.print_data}")
        return "done"
