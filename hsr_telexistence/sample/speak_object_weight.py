#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (C) 2016 Toyota Motor Corporation
"""Speak Object Weight Sample"""

import math
import os
import sys

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import WrenchStamped
import rospy
from sensor_msgs.msg import JointState
from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal
)
from tmc_manipulation_msgs.srv import (
    SafeJointChange,
    SafeJointChangeRequest
)
from tmc_msgs.msg import (
    TalkRequestAction,
    TalkRequestGoal,
    Voice
)

_CONNECTION_TIMEOUT = 10.0


_force_data_x = 0.0
_force_data_y = 0.0
_force_data_z = 0.0
_torque_data_x = 0.0
_torque_data_y = 0.0
_torque_data_z = 0.0

def __ft_sensor_cb(data):
    global _force_data_x, _force_data_y, _force_data_z, _torque_data_x, _torque_data_y, _torque_data_z
    _force_data_x = data.wrench.force.x
    _force_data_y = data.wrench.force.y
    _force_data_z = data.wrench.force.z
    _torque_data_z = data.wrench.torque.x
    _torque_data_y = data.wrench.torque.y
    _torque_data_z = data.wrench.torque.z

def main():
    global _force_data_x, _force_data_y, _force_data_z, _torque_data_x, _torque_data_y, _torque_data_z

    # Subscribe force torque sensor data from HSRB
    ft_sensor_topic = '/hsrb/wrist_wrench/raw'
    _wrist_wrench_sub = rospy.Subscriber(
        ft_sensor_topic, WrenchStamped, __ft_sensor_cb)

    # Wait for connection
    try:
        rospy.wait_for_message(ft_sensor_topic, WrenchStamped,
                                timeout=_CONNECTION_TIMEOUT)
    except Exception as e:
        rospy.logerr(e)
        sys.exit(1)
    while not rospy.is_shutdown():
        a = _force_data_x **2 + _force_data_y ** 2 + _force_data_z ** 2
        b = _torque_data_x **2 + _torque_data_y ** 2 + _torque_data_z ** 2
        # print("force" + str(a))
        # print("")
        print("torque" + str(b))
        # print("")

    # # Start force sensor capture
    # force_sensor_capture = ForceSensorCapture()

    # # Set initial pose
    # joint_controller = JointController()

    # initial_position = JointState()
    # initial_position.name.extend(['arm_lift_joint', 'arm_flex_joint',
    #                               'arm_roll_joint', 'wrist_flex_joint',
    #                               'wrist_roll_joint', 'head_pan_joint',
    #                               'head_tilt_joint', 'hand_motor_joint'])
    # initial_position.position.extend([0.0, 0.0, 0.0, -1.57,
    #                                   0.0, 0.0, 0.0, 1.2])
    # joint_controller.move_to_joint_positions(initial_position)

    # # Get initial data of force sensor
    # pre_force_list = force_sensor_capture.get_current_force()

    # # Ask user to set object
    # speaker = Speaker()
    # speaker.speak_sentence(_EXPLAIN1[speaker.get_language()])
    # rospy.sleep(2.0)

    # # Inform user of next gripper action
    # speaker.speak_sentence(_EXPLAIN2[speaker.get_language()])
    # rospy.sleep(1.0)

    # # Grasp the object
    # joint_controller.grasp(-0.1)

    # # Wait until force sensor data become stable
    # rospy.sleep(1.0)
    # post_force_list = force_sensor_capture.get_current_force()

    # force_difference = compute_difference(pre_force_list, post_force_list)

    # # Convert newton to gram
    # weight = round(force_difference / 9.81 * 1000, 1)

    # # Speak object weight in first decimal place
    # speaker.speak_sentence(_ANSWER[speaker.get_language()].format(weight))


if __name__ == '__main__':
    rospy.init_node('hsrb_speak_object_weight')
    main()
