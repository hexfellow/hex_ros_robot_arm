#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

from collections import deque
from typing import Any, Optional
from abc import abstractmethod

from hex_ros_common.utility import InterfaceBase

from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboManipCtrlStamped,
    HexDcRoboManipStateStamped,
)
from hex_util_msg.dataclass.dataclass_teleop import (
    HexDcTeleopHandleStateStamped,
)

JOINT_STATE_NAME = [f"joint_{i}" for i in range(1, 7)] + ["grip_joint_1"]


class ArmInterfaceBase(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super().__init__(name)

        ### ros parameters
        self._rate_param = {}
        self._robot_param = {}

        ### rx msg queues
        self._manip_ctrl_deque = deque(maxlen=100)
        self._color_cmd_deque = deque(maxlen=10)

    ####################
    ### parameters
    ####################
    def get_rate_param(self) -> dict:
        return self._rate_param

    def get_robot_param(self) -> dict:
        return self._robot_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_manip_state(self, out: HexDcRoboManipStateStamped):
        raise NotImplementedError("ArmInterfaceBase.pub_manip_state")

    @abstractmethod
    def pub_joint_state(self, out: HexDcRoboManipStateStamped):
        raise NotImplementedError("ArmInterfaceBase.pub_joint_state")

    @abstractmethod
    def pub_clock(self, stamp_ns: int):
        raise NotImplementedError("ArmInterfaceBase.pub_clock")

    @abstractmethod
    def pub_joy_state(self, out: HexDcTeleopHandleStateStamped):
        raise NotImplementedError("ArmInterfaceBase.pub_joy_state")

    ####################
    ### subscribers
    ####################
    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        if not latest:
            if dq:
                return dq.popleft()
            else:
                return None
        else:
            if dq:
                ret = dq[-1]
                dq.clear()
                return ret
            else:
                return None

    # manip ctrl
    def get_manip_ctrl(
        self,
        latest: bool = False,
    ) -> Optional[HexDcRoboManipCtrlStamped]:
        return self.deque_helper(self._manip_ctrl_deque, latest)

    # color command (RGB LED — Hello Y6)
    def get_color_cmd(self, latest: bool = False) -> Optional[dict[str, list[int]]]:
        return self.deque_helper(self._color_cmd_deque, latest)
