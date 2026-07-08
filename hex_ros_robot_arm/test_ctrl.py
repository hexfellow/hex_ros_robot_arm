#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import copy, os, sys, time
import numpy as np

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntFull,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl,
    HexDcRoboArmCtrlMode as ArmCtrlMode,
    HexDcRoboGripCtrl,
    HexDcRoboGripCtrlMode as GripCtrlMode,
    HexDcRoboManipCtrl,
)

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from test_utils import DataInterface

# ---------------------------------------------------------------------------
# constants
# ---------------------------------------------------------------------------
ARM_DOF = 6
GRIP_DOF = 1

# joint position presets for arm (MIT / JNT modes)
ARM_POS_HOME = [0.0, -1.5, 3.0, 0.0, 0.0, 0.0]
ARM_POS_EXTEND = [0.5, -1.0, 1.5, -0.5, 0.5, 0.0]
ARM_POS_RETRACT = [-0.5, 0.5, 1.57, -0.5, 0.5, 0.0]
ARM_POS_PRESETS = [ARM_POS_HOME, ARM_POS_EXTEND, ARM_POS_RETRACT]

# end-effector pose presets for arm (EE mode) — [position], [quaternion wxyz]
ARM_EE_POSE_A = ([0.3, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0])
ARM_EE_POSE_B = ([0.3, 0.2, 0.3], [1.0, 0.0, 0.0, 0.0])
ARM_EE_PRESETS = [ARM_EE_POSE_A, ARM_EE_POSE_B]

# grip position presets (MIT mode)
GRIP_POS_OPEN = [0.0]
GRIP_POS_CLOSE = [0.5]
GRIP_POS_PRESETS = [GRIP_POS_OPEN, GRIP_POS_CLOSE]

# grip effort presets (JNT / TAU modes)
GRIP_EFF_OPEN = [-1.0]
GRIP_EFF_CLOSE = [1.0]
GRIP_EFF_PRESETS = [GRIP_EFF_OPEN, GRIP_EFF_CLOSE]

# default gains — consistent with robot driver defaults
ARM_KP = [200.0, 200.0, 200.0, 200.0, 100.0, 100.0]
ARM_KD = [5.0, 5.0, 5.0, 5.0, 2.0, 2.0]
ARM_LIM_VEL = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
ARM_LIM_ACC = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]

# default mit
ARM_MIT_KP = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
ARM_MIT_KD = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
ARM_MIT_LIM_VEL = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ARM_MIT_LIM_ACC = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

GRIP_KP = [10.0]
GRIP_KD = [0.5]
GRIP_LIM_VEL = [0.5]
GRIP_LIM_ACC = [1.0]

# how long to hold each preset before advancing [s]
CYCLE_PERIOD = 5.0

# ---------------------------------------------------------------------------
# helpers – construct control dataclass fields
# ---------------------------------------------------------------------------

DEFAULT_ARM_JNT_FULL = HexDcBaseJntFull(
    pos=np.zeros(ARM_DOF, dtype=np.float64),
    vel=np.zeros(ARM_DOF, dtype=np.float64),
    eff=np.zeros(ARM_DOF, dtype=np.float64),
    kp=np.asarray(ARM_KP, dtype=np.float64),
    kd=np.asarray(ARM_KD, dtype=np.float64),
    lim_vel=np.asarray(ARM_LIM_VEL, dtype=np.float64),
    lim_acc=np.asarray(ARM_LIM_ACC, dtype=np.float64),
)

DEFAULT_ARM_MIT_FULL = HexDcBaseJntFull(
    pos=np.zeros(ARM_DOF, dtype=np.float64),
    vel=np.zeros(ARM_DOF, dtype=np.float64),
    eff=np.zeros(ARM_DOF, dtype=np.float64),
    kp=np.asarray(ARM_MIT_KP, dtype=np.float64),
    kd=np.asarray(ARM_MIT_KD, dtype=np.float64),
    lim_vel=np.asarray(ARM_MIT_LIM_VEL, dtype=np.float64),
    lim_acc=np.asarray(ARM_MIT_LIM_ACC, dtype=np.float64),
)
DEFAULT_ARM_POSE = HexDcBasePose(
    position=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
    orientation=HexDcBaseQuaternion(x=0.0, y=0.0, z=0.0, w=1.0),
)
DEFAULT_GRIP_JNT_FULL = HexDcBaseJntFull(
    pos=np.zeros(GRIP_DOF, dtype=np.float64),
    vel=np.zeros(GRIP_DOF, dtype=np.float64),
    eff=np.zeros(GRIP_DOF, dtype=np.float64),
    kp=np.asarray(GRIP_KP, dtype=np.float64),
    kd=np.asarray(GRIP_KD, dtype=np.float64),
    lim_vel=np.asarray(GRIP_LIM_VEL, dtype=np.float64),
    lim_acc=np.asarray(GRIP_LIM_ACC, dtype=np.float64),
)

DEFAULT_ARM_CTRL = HexDcRoboArmCtrl(
    ctrl_mode=ArmCtrlMode.NONE,
    grav=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
    jnt=DEFAULT_ARM_JNT_FULL,
    pose=DEFAULT_ARM_POSE,
)

DEFAULT_GRIP_CTRL = HexDcRoboGripCtrl(
    ctrl_mode=GripCtrlMode.NONE,
    jnt=DEFAULT_GRIP_JNT_FULL,
)


def none_case(cycle_idx: int) -> HexDcRoboManipCtrl:
    return HexDcRoboManipCtrl(
        arm_ctrl=DEFAULT_ARM_CTRL,
        grip_ctrl=DEFAULT_GRIP_CTRL,
    )


def mit_case(cycle_idx: int, grav: bool = True) -> HexDcRoboManipCtrl:
    mit_arm_ctrl = copy.deepcopy(DEFAULT_ARM_CTRL)
    mit_arm_ctrl.ctrl_mode = ArmCtrlMode.MIT
    mit_arm_ctrl.jnt = copy.deepcopy(DEFAULT_ARM_MIT_FULL)
    mit_arm_ctrl.jnt.pos = ARM_POS_PRESETS[cycle_idx % len(ARM_POS_PRESETS)]
    if grav:
        mit_arm_ctrl.grav.z = -9.81
    mit_grip_ctrl = copy.deepcopy(DEFAULT_GRIP_CTRL)
    mit_grip_ctrl.ctrl_mode = GripCtrlMode.MIT
    mit_grip_ctrl.jnt.pos = GRIP_POS_PRESETS[cycle_idx % len(GRIP_POS_PRESETS)]
    return HexDcRoboManipCtrl(
        arm_ctrl=mit_arm_ctrl,
        grip_ctrl=mit_grip_ctrl,
    )


def jnt_case(cycle_idx: int, grav: bool = True) -> HexDcRoboManipCtrl:
    jnt_arm_ctrl = copy.deepcopy(DEFAULT_ARM_CTRL)
    jnt_arm_ctrl.ctrl_mode = ArmCtrlMode.JNT
    jnt_arm_ctrl.jnt.pos = ARM_POS_PRESETS[cycle_idx % len(ARM_POS_PRESETS)]
    if grav:
        jnt_arm_ctrl.grav.x = 0.0
        jnt_arm_ctrl.grav.y = 0.0
        jnt_arm_ctrl.grav.z = -9.81
    jnt_grip_ctrl = copy.deepcopy(DEFAULT_GRIP_CTRL)
    jnt_grip_ctrl.ctrl_mode = GripCtrlMode.JNT
    jnt_grip_ctrl.jnt.pos = GRIP_POS_PRESETS[cycle_idx % len(GRIP_POS_PRESETS)]
    jnt_grip_ctrl.jnt.eff = GRIP_EFF_PRESETS[cycle_idx % len(GRIP_EFF_PRESETS)]
    return HexDcRoboManipCtrl(
        arm_ctrl=jnt_arm_ctrl,
        grip_ctrl=jnt_grip_ctrl,
    )


def ee_case(cycle_idx: int, grav: bool = True) -> HexDcRoboManipCtrl:
    ee_arm_ctrl = copy.deepcopy(DEFAULT_ARM_CTRL)
    ee_arm_ctrl.ctrl_mode = ArmCtrlMode.EE
    pos, ori = ARM_EE_PRESETS[cycle_idx % len(ARM_EE_PRESETS)]
    ee_arm_ctrl.pose.position = HexDcBaseVector3(x=pos[0], y=pos[1], z=pos[2])
    ee_arm_ctrl.pose.orientation = HexDcBaseQuaternion(w=ori[0],
                                                       x=ori[1],
                                                       y=ori[2],
                                                       z=ori[3])
    if grav:
        ee_arm_ctrl.grav.x = 0.0
        ee_arm_ctrl.grav.y = 0.0
        ee_arm_ctrl.grav.z = -9.81
    ee_grip_ctrl = copy.deepcopy(DEFAULT_GRIP_CTRL)
    ee_grip_ctrl.ctrl_mode = GripCtrlMode.TAU
    ee_grip_ctrl.jnt.eff = GRIP_EFF_PRESETS[cycle_idx % len(GRIP_EFF_PRESETS)]
    return HexDcRoboManipCtrl(
        arm_ctrl=ee_arm_ctrl,
        grip_ctrl=ee_grip_ctrl,
    )


class TestCtrl:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("test_ctrl")

        ### parameters
        self._rate_param = self.__data_interface.get_rate_param()
        self.__freq = self._rate_param["ros"]
        self.__data_interface.logi(f"freq: {self.__freq} hz")

        ### derived
        self.__cycle_decim = max(1, int(round(CYCLE_PERIOD * self.__freq)))

        ### derived
        self.__ctrl_decim = max(
            1, int(round(self.__freq / self._rate_param["ctrl"])))

    def run(self):
        step = 0
        ctrl_count = 0
        state_count = 0
        report_time = time.monotonic()
        while self.__data_interface.ok():
            ctrl_count += 1
            if ctrl_count >= self.__ctrl_decim:
                ctrl_count = 0
                # 1. advance the preset cycle every CYCLE_PERIOD seconds
                cycle_idx = step // self.__cycle_decim

                # 2. build the manipulator control command
                # manip_ctrl = none_case(cycle_idx)
                # manip_ctrl = mit_case(cycle_idx)
                manip_ctrl = jnt_case(cycle_idx)
                # manip_ctrl = ee_case(cycle_idx)

                # 3. publish the control command
                self.__data_interface.pub_manip_ctrl(manip_ctrl)

            # 4. drain all received manip_state messages
            while self.__data_interface.get_manip_state() is not None:
                state_count += 1

            # 5. report the manip_state receive frequency once per second
            now = time.monotonic()
            elapsed = now - report_time
            if elapsed >= 1.0:
                self.__data_interface.logi(
                    f"manip_state freq: {state_count / elapsed:.1f} hz")
                state_count = 0
                report_time = now

            step += 1
            self.__data_interface.sleep()

    def shutdown(self):
        try:
            self.__data_interface.shutdown()
        except Exception:
            pass


def main():
    test_ctrl = TestCtrl()
    try:
        test_ctrl.run()
    except KeyboardInterrupt:
        pass
    finally:
        test_ctrl.shutdown()


if __name__ == '__main__':
    main()