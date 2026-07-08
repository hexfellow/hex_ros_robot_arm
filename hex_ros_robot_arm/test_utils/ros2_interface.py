#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import numpy as np
import threading

import rclpy
import rclpy.node

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from sensor_msgs.msg import JointState
from hex_ros_msgs.msg import (
    HexRosJnt,
    HexRosRoboArmCtrl,
    HexRosRoboGripCtrl,
    HexRosRoboManipCtrl,
    HexRosRoboManipCtrlStamped,
    HexRosRoboManipStateStamped,
)

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntState,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl,
    HexDcRoboArmState,
    HexDcRoboGripCtrl,
    HexDcRoboGripState,
    HexDcRoboManipCtrl,
    HexDcRoboManipState,
    HexDcRoboManipStateStamped,
)

from .interface_base import TestInterfaceBase


class DataInterface(TestInterfaceBase):

    def __init__(self, name: str = "unknown"):
        rclpy.init()
        self._node = rclpy.node.Node(name)
        self._logger = self._node.get_logger()
        self._shutting_down = False
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        super().__init__(name)

        ### parameters
        self._node.declare_parameter('rate_ros', 1000.0)
        self._rate_param["ros"] = self._node.get_parameter('rate_ros').value
        self.__rate = self._node.create_rate(self._rate_param["ros"])

        self._node.declare_parameter('rate_ctrl', 500.0)
        self._rate_param.update({
            "ctrl": self._node.get_parameter('rate_ctrl').value,
        })

        ### publisher
        self.__manip_ctrl_pub = self._node.create_publisher(
            HexRosRoboManipCtrlStamped,
            'manip_ctrl',
            10,
        )

        ### subscriber
        self.__manip_state_sub = self._node.create_subscription(
            HexRosRoboManipStateStamped,
            'manip_state',
            self.__manip_state_callback,
            10,
        )
        self.__manip_state_sub

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### ros infrastructure
    ####################
    def ok(self) -> bool:
        return rclpy.ok()

    def shutdown(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.__spin_thread.join()

    def __spin(self):
        try:
            rclpy.spin(self._node)
        except rclpy.executors.ExternalShutdownException:
            pass

    def now_ns(self) -> int:
        return self._node.get_clock().now().nanoseconds

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        self._logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self._logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self._logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self._logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self._logger.fatal(msg, *args, **kwargs)

    ####################
    ### publishers
    ####################
    def pub_manip_ctrl(self, out: HexDcRoboManipCtrl):
        msg = HexRosRoboManipCtrlStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.manip_ctrl = HexRosRoboManipCtrl(
            arm_ctrl=self.__arm_ctrl_to_msg(out.arm_ctrl),
            grip_ctrl=self.__grip_ctrl_to_msg(out.grip_ctrl),
        )
        self.__manip_ctrl_pub.publish(msg)

    @staticmethod
    def __jnt_to_msg(jnt) -> HexRosJnt:
        return HexRosJnt(
            pos=np.asarray(jnt.pos, dtype=np.float64).tolist(),
            vel=np.asarray(jnt.vel, dtype=np.float64).tolist(),
            eff=np.asarray(jnt.eff, dtype=np.float64).tolist(),
            kp=np.asarray(jnt.kp, dtype=np.float64).tolist(),
            kd=np.asarray(jnt.kd, dtype=np.float64).tolist(),
            lim_vel=np.asarray(jnt.lim_vel, dtype=np.float64).tolist(),
            lim_acc=np.asarray(jnt.lim_acc, dtype=np.float64).tolist(),
        )

    @staticmethod
    def __arm_ctrl_to_msg(arm: HexDcRoboArmCtrl) -> HexRosRoboArmCtrl:
        return HexRosRoboArmCtrl(
            ctrl_mode=int(arm.ctrl_mode),
            grav=Vector3(x=arm.grav.x, y=arm.grav.y, z=arm.grav.z),
            jnt=DataInterface.__jnt_to_msg(arm.jnt),
            pose=Pose(
                position=Point(
                    x=arm.pose.position.x,
                    y=arm.pose.position.y,
                    z=arm.pose.position.z,
                ),
                orientation=Quaternion(
                    x=arm.pose.orientation.x,
                    y=arm.pose.orientation.y,
                    z=arm.pose.orientation.z,
                    w=arm.pose.orientation.w,
                ),
            ),
        )

    @staticmethod
    def __grip_ctrl_to_msg(grip: HexDcRoboGripCtrl) -> HexRosRoboGripCtrl:
        return HexRosRoboGripCtrl(
            ctrl_mode=int(grip.ctrl_mode),
            jnt=DataInterface.__jnt_to_msg(grip.jnt),
        )

    ####################
    ### subscribers
    ####################
    def __manip_state_callback(self, msg: HexRosRoboManipStateStamped):
        self._manip_state_deque.append(self.__manip_state_msg_to_dc(msg))

    @staticmethod
    def __jnt_state_to_dc(jnt: JointState) -> HexDcBaseJntState:
        return HexDcBaseJntState(
            position=np.asarray(jnt.position, dtype=np.float64),
            velocity=np.asarray(jnt.velocity, dtype=np.float64),
            effort=np.asarray(jnt.effort, dtype=np.float64),
        )

    @staticmethod
    def __pose_to_dc(pose: Pose) -> HexDcBasePose:
        return HexDcBasePose(
            position=HexDcBaseVector3(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
            ),
            orientation=HexDcBaseQuaternion(
                x=pose.orientation.x,
                y=pose.orientation.y,
                z=pose.orientation.z,
                w=pose.orientation.w,
            ),
        )

    @staticmethod
    def __manip_state_msg_to_dc(
            msg: HexRosRoboManipStateStamped) -> HexDcRoboManipStateStamped:
        header = HexDcBaseHeader(
            stamp=HexDcBaseTime(
                secs=int(msg.header.stamp.sec),
                nsecs=int(msg.header.stamp.nanosec),
            ),
            frame_id=msg.header.frame_id,
        )

        arm_msg = msg.manip_state.arm_state
        arm_state = HexDcRoboArmState(
            jnt=DataInterface.__jnt_state_to_dc(arm_msg.jnt),
            pose=DataInterface.__pose_to_dc(arm_msg.pose),
        )

        grip_msg = msg.manip_state.grip_state
        grip_state = HexDcRoboGripState(jnt=DataInterface.__jnt_state_to_dc(
            grip_msg.jnt), )

        return HexDcRoboManipStateStamped(
            header=header,
            manip_state=HexDcRoboManipState(
                arm_state=arm_state,
                grip_state=grip_state,
            ),
        )
