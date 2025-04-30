#!/usr/bin/env python3
# coding: utf-8
import sys
from math import cos, sin, pi
from copy import deepcopy
from geometry_msgs.msg import Pose
import rospy
import moveit_commander

# 初始化
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("circular_path_demo", anonymous=True)
arm = moveit_commander.MoveGroupCommander("manipulator")

# 起始點：當前姿態
start_pose = arm.get_current_pose().pose
waypoints = []

# 圓心設定（YZ平面）
center_y = start_pose.position.y
center_z = start_pose.position.z
radius = 0.2

# 原點也加進去
waypoints.append(deepcopy(start_pose))

# 生成圓弧點（0 ~ 2pi）
for th in [i * 0.1 for i in range(63)]:
    p = deepcopy(start_pose)
    p.position.y = center_y + radius * cos(th)
    p.position.z = center_z + radius * sin(th)
    waypoints.append(p)

# 執行 cartesian 規劃
(plan, fraction) = arm.compute_cartesian_path(
    waypoints,
    eef_step = 0.02 ,     # eef_step
    avoid_collisions=True
)

# 手動補上時間戳（ROS Noetic 無 ITP）
for i, pt in enumerate(plan.joint_trajectory.points):
    pt.time_from_start = rospy.Duration((i + 1) * 0.1)

# 執行
if fraction > 0.9:
    rospy.loginfo("成功率: %.2f%%，執行圓弧路徑中...", fraction * 100)
    arm.execute(plan, wait=True)
else:
    rospy.logwarn("Cartesian path 規劃失敗，成功率太低：%.2f%%", fraction * 100)

moveit_commander.roscpp_shutdown()
