#!/usr/bin/env python3
# coding: utf-8

import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_to_up_pose", anonymous=True)
arm = moveit_commander.MoveGroupCommander("manipulator")

# 設定為 SRDF 裡定義的 'up' 姿態
arm.set_named_target("up")

# 執行移動
plan = arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()

moveit_commander.roscpp_shutdown()