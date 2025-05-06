#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from copy import deepcopy

def move_to_pose(arm, plan, fraction , criterion = 0.8):
    if fraction >= criterion:
        # 手動補上時間資訊
        duration_per_point = 0.05
        for i, point in enumerate(plan.joint_trajectory.points):
            point.time_from_start = rospy.Duration((i + 1) * duration_per_point)

        rospy.loginfo("Total points in trajectory: %d", len(plan.joint_trajectory.points))
        for i, point in enumerate(plan.joint_trajectory.points):
            rospy.loginfo("Point %d:", i)
            rospy.loginfo("  Time from start: %.4f s", point.time_from_start.to_sec())
            rospy.loginfo("  Positions: %s", [round(p, 4) for p in point.positions])

        rospy.loginfo("Executing Cartesian path...")
        arm.execute(plan, wait=True)
    else:
        rospy.logwarn("Cartesian path planning failed. Fraction too low.")


def move_above_table(arm, pose_start, table_pose, table_size, cube_size):
    """
    [PLACE] Step 1 : 
    """
    # 建立路徑點：先 current，再上方 pose1
    _, _, cube_size_z = cube_size
    _, _, table_size_z = table_size

    pose_above_table = deepcopy(pose_start)
    pose_above_table.position.z = 0.8
    pose_above_table.position.x = table_pose.position.x
    pose_above_table.position.y = table_pose.position.y 

    # 計算 Cartesian path
    plan_lift, fraction_lift = arm.compute_cartesian_path([pose_start, pose_above_table], 0.03, True)

    rospy.loginfo("[PICK] Cartesian coverage above object: %.2f%%", fraction_lift * 100)

    # 執行
    if fraction_lift > 0.5 :
        move_to_pose(arm, plan_lift, fraction_lift , criterion=0.5)
        return pose_above_table
    else:
        rospy.logwarn("[PICK] move_above_object failed (%.2f%%)", fraction_lift * 100)
        return None


# def down_to_object(arm, pose_above, cube_pose, cube_size_z):
#     """
#     [PICK] Step 3: 從上方 pose 往下移到抓取高度。
#     回傳新的 Pose（成功）或 None（失敗）。
#     """
#     waypoints = [deepcopy(pose_above)]
#     pose_down = deepcopy(pose_above)
#     gripper_pad_thickness = 0.01
#     pose_down.position.z = cube_pose.position.z + (cube_size_z / 2) - gripper_pad_thickness
#     waypoints.append(deepcopy(pose_down))

#     (plan, fraction) =  arm.compute_cartesian_path(waypoints, 0.02, True)
#     rospy.loginfo("[PICK] Cartesian coverage downward: %.2f%%", fraction * 100)

#     if fraction > 0.3 :
#         move_to_pose(arm, plan, fraction, criterion = 0.3)
#         return pose_down
#     else:
#         rospy.logwarn("[PICK] down_to_object failed (%.2f%%)", fraction * 100)
#         return None
