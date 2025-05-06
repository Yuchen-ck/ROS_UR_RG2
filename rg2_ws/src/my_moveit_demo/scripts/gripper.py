#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from copy import deepcopy

# 目標值與容忍誤差設定
TARGET_OPEN = 0.005
TARGET_CLOSE = -0.05
OPEN_TOLERANCE = 0.1
CLOSE_TOLERANCE = 0.5

# 全域變數：從 joint_states callback 更新
gripper_position = None

def joint_states_callback(msg: JointState):
    """從 /ur5/joint_states 擷取 gripper_joint 的位置"""
    global gripper_position
    try:
        idx = msg.name.index('gripper_joint')
        gripper_position = msg.position[idx]
    except ValueError:
        # 如果目前訊息裡沒有 gripper_joint，就跳過
        rospy.logwarn_throttle(10.0, "'gripper_joint' not in JointState")

# 一開始就訂閱，保證後面呼叫 open/close 前能收到位置
rospy.Subscriber('/ur5/joint_states', JointState, joint_states_callback)

def check_success(target: float, tol: float, timeout: float = 5.0) -> bool:
    """
    等待 gripper_joint 去到 target ± tol。
    timeout 秒內沒到就回 False。
    """
    rate = rospy.Rate(10)
    start = rospy.get_time()
    while rospy.get_time() - start < timeout:
        if gripper_position is not None:
            diff = abs(gripper_position - target)
            rospy.loginfo("Checking gripper: current=%.3f, target=%.3f, diff=%.3f",
                          gripper_position, target, diff)
            if diff < tol:
                return True
        rate.sleep()
    return False

def open_gripper(gripper) -> bool:
    """打開夾爪，並確認到位後回 True，失敗回 False"""
    gripper.set_joint_value_target({"gripper_joint": 0.5})
    success = gripper.go(wait=True)
    gripper.stop()
    if not success:
        rospy.logwarn("Failed to send open_gripper command")
    else:
        rospy.loginfo("Open command sent, waiting to reach %.3f", TARGET_OPEN)
    
    if check_success(TARGET_OPEN, OPEN_TOLERANCE):
        rospy.loginfo("Gripper opened successfully")
        return True
    else:
        rospy.logerr("Gripper did NOT reach open target")
        return False

def close_gripper(gripper) -> bool:
    """閉合夾爪，並確認到位後回 True，失敗回 False"""
    gripper.set_joint_value_target({"gripper_joint": TARGET_CLOSE})
    success = gripper.go(wait=True)
    gripper.stop()
    if not success:
        rospy.logwarn("Failed to send close_gripper command")
    else:
        rospy.loginfo("Close command sent, waiting to reach %.3f", TARGET_CLOSE)
    
    if check_success(TARGET_CLOSE, CLOSE_TOLERANCE):
        rospy.loginfo("Gripper closed successfully")
        return True
    else:
        rospy.logerr("Gripper did NOT reach close target")
        return False
